#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_INA3221.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Configuration constants
#define SD_CS     5
#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23
#define WAKE_PIN  15
#define MAX_RETRIES 5
#define FLUSH_INTERVAL_MS 2000
#define LOOP_DELAY_MS 20

// Sleep configuration
#define SLEEP_DURATION_SEC 5
#define MOVEMENT_THRESHOLD 0.5f
#define BARO_SAMPLE_INTERVAL 50

// Display configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Sensor value ranges for validation
#define ACCEL_MIN -2000.0f
#define ACCEL_MAX  2000.0f
#define GYRO_MIN  -2000.0f
#define GYRO_MAX   2000.0f
#define ALT_MIN   -500.0f
#define ALT_MAX    10000.0f
#define TEMP_MIN  -50.0f
#define TEMP_MAX   100.0f

// Error codes
#define ERROR_NONE 0
#define ERROR_SD_INIT 1
#define ERROR_IMU_INIT 2
#define ERROR_BARO_INIT 3
#define ERROR_FILE_OPEN 4
#define ERROR_SENSOR_RANGE 5
#define ERROR_WRITE_FAILED 6
#define ERROR_DISPLAY_INIT 7

// Accelerometer calibration
struct AccelCal {
  float offset_x;
  float offset_y;
  float offset_z;
  float scale_x;
  float scale_y;
  float scale_z;
} accel_cal = {-40.5f, -5.0f, 16.0f, 1.0f, 1.0f, 1.0f};

// Sensor data structure
struct SensorData {
  unsigned long timestamp;
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float altitude;
  float temperature;
};

// Sleep state tracking
struct SleepState {
  float last_ax;
  float last_ay;
  float last_az;
  bool first_read;
  int stable_count;
} sleep_state = {0.0f, 0.0f, 0.0f, true, 0};

// Global objects
SPIClass spi(VSPI);
ICM_20948_I2C myICM;
Adafruit_MPL3115A2 baro;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
File logFile;

bool icm_ok = false;
bool mpl_ok = false;
bool display_ok = false;
int error_state = ERROR_NONE;
unsigned long session_start_time = 0;
unsigned long loop_count = 0;

// Rule 5: Assertion for recovery
bool assertRange(float value, float min, float max, int errorCode) {
  if (value < min || value > max) {
    error_state = errorCode;
    Serial.print("Range error: ");
    Serial.print(value);
    Serial.print(" not in [");
    Serial.print(min);
    Serial.print(", ");
    Serial.print(max);
    Serial.println("]");
    return false;
  }
  return true;
}

// Rule 5: Assertion for file operations
bool assertFileValid(void) {
  if (!logFile) {
    error_state = ERROR_WRITE_FAILED;
    Serial.println("File not valid!");
    return false;
  }
  return true;
}

// Rule 2 & 4: Bounded display initialization
bool initDisplay(void) {
  Serial.println("Initializing display...");
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Display Ready");
      display.display();
      Serial.println("Display OK!");
      return true;
    }
    Serial.print("Display retry ");
    Serial.println(retry);
    delay(100);
  }
  error_state = ERROR_DISPLAY_INIT;
  Serial.println("Display FAILED!");
  return false;
}

bool initSD(void) {
  Serial.println("Initializing SD...");
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (SD.begin(SD_CS, spi)) {
      Serial.println("SD OK!");
      return true;
    }
    Serial.print("SD retry ");
    Serial.println(retry);
    delay(100);
  }
  error_state = ERROR_SD_INIT;
  Serial.println("SD FAILED!");
  return false;
}

bool initIMU(void) {
  Serial.println("Initializing IMU...");
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    myICM.begin(Wire, 0x68);
    if (myICM.status == ICM_20948_Stat_Ok) {
      Serial.println("IMU OK at 0x68!");
      return true;
    }
    
    myICM.begin(Wire, 0x69);
    if (myICM.status == ICM_20948_Stat_Ok) {
      Serial.println("IMU OK at 0x69!");
      return true;
    }
    
    Serial.print("IMU retry ");
    Serial.println(retry);
    delay(100);
  }
  error_state = ERROR_IMU_INIT;
  Serial.println("IMU FAILED!");
  return false;
}

bool initBarometer(void) {
  Serial.println("Initializing Barometer...");
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (baro.begin()) {
      baro.setSeaPressure(1013.26);
      
      // Set to OSR=0 (fastest mode)
      Wire.beginTransmission(0x60);
      Wire.write(0x26);
      Wire.write(0x39);
      Wire.endTransmission();
      
      delay(20);
      
      // Verify
      Wire.beginTransmission(0x60);
      Wire.write(0x26);
      Wire.endTransmission(false);
      Wire.requestFrom(0x60, 1);
      if (Wire.available()) {
        uint8_t ctrl_reg = Wire.read();
        Serial.print("Barometer CTRL_REG1 = 0x");
        Serial.println(ctrl_reg, HEX);
      }
      
      Serial.println("Barometer OK!");
      return true;
    }
    Serial.print("Barometer retry ");
    Serial.println(retry);
    delay(100);
  }
  error_state = ERROR_BARO_INIT;
  Serial.println("Barometer FAILED!");
  return false;
}

void getCompileDateTime(char* buffer, size_t bufferSize) {
  snprintf(buffer, bufferSize, "Session started (compile time EST): %s %s", __DATE__, __TIME__);
}

bool initLogFile(void) {
  Serial.println("Initializing log file...");
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    logFile = SD.open("/imu_log.csv", FILE_WRITE);
    if (logFile) {
      char datetime[80];
      getCompileDateTime(datetime, sizeof(datetime));
      logFile.print("# ");
      logFile.println(datetime);
      
      size_t written = logFile.println("timestamp_ms,ax,ay,az,gx,gy,gz,mx,my,mz,alt_m,temp_C");
      if (written > 0) {
        logFile.flush();
        session_start_time = millis();
        Serial.print("Log file OK! Header bytes: ");
        Serial.println(written);
        return true;
      }
      logFile.close();
    }
    Serial.print("Log file retry ");
    Serial.println(retry);
    delay(100);
  }
  error_state = ERROR_FILE_OPEN;
  Serial.println("Log file FAILED!");
  return false;
}

bool readIMUSensors(SensorData* data) {
  if (data == NULL) {
    error_state = ERROR_SENSOR_RANGE;
    return false;
  }
  
  if (!icm_ok) {
    return false;
  }
  
  myICM.getAGMT();
  if (myICM.status != ICM_20948_Stat_Ok) {
    return false;
  }
  
  data->ax = (myICM.accX() - accel_cal.offset_x) * accel_cal.scale_x;
  data->ay = (myICM.accY() - accel_cal.offset_y) * accel_cal.scale_y;
  data->az = (myICM.accZ() - accel_cal.offset_z) * accel_cal.scale_z;
  
  data->gx = myICM.gyrX();
  data->gy = myICM.gyrY();
  data->gz = myICM.gyrZ();
  data->mx = myICM.magX();
  data->my = myICM.magY();
  data->mz = myICM.magZ();
  
  if (!assertRange(data->ax, ACCEL_MIN, ACCEL_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->ay, ACCEL_MIN, ACCEL_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->az, ACCEL_MIN, ACCEL_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->gx, GYRO_MIN, GYRO_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->gy, GYRO_MIN, GYRO_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->gz, GYRO_MIN, GYRO_MAX, ERROR_SENSOR_RANGE)) return false;
  
  return true;
}

// Read barometer with timeout protection
bool readBarometer(SensorData* data) {
  if (data == NULL || !mpl_ok) {
    return false;
  }
  
  // Set default values in case of failure
  data->altitude = 0.0f;
  data->temperature = 25.0f;
  
  unsigned long start = millis();
  
  // Try to read altitude with timeout
  data->altitude = baro.getAltitude();
  if (millis() - start > 50) {  // 50ms timeout
    Serial.println("Barometer altitude timeout!");
    return false;
  }
  
  // Try to read temperature with timeout
  start = millis();
  data->temperature = baro.getTemperature();
  if (millis() - start > 50) {  // 50ms timeout
    Serial.println("Barometer temperature timeout!");
    return false;
  }
  
  // Skip range validation for now - just return success
  return true;
}

bool writeLogData(const SensorData* data) {
  if (data == NULL) {
    Serial.println("writeLogData: NULL data");
    return false;
  }
  
  if (!assertFileValid()) {
    return false;
  }
  
  char line[160];
  int len = snprintf(line, sizeof(line),
           "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f",
           data->timestamp, data->ax, data->ay, data->az,
           data->gx, data->gy, data->gz,
           data->mx, data->my, data->mz,
           data->altitude, data->temperature);
  
  if (len < 0 || len >= (int)sizeof(line)) {
    error_state = ERROR_WRITE_FAILED;
    Serial.println("snprintf overflow!");
    return false;
  }
  
  size_t written = logFile.println(line);
  if (written == 0) {
    error_state = ERROR_WRITE_FAILED;
    Serial.println("Write failed - 0 bytes!");
    return false;
  }
  
  return true;
}

void updateDisplay(const SensorData* data) {
  if (!display_ok || data == NULL) {
    return;
  }
  
  display.clearDisplay();
  display.setCursor(0, 0);
  
  display.print("LOOP:");
  display.println(loop_count);
  
  display.print("A:");
  display.print(data->ax, 1);
  display.print(",");
  display.print(data->ay, 1);
  display.print(",");
  display.println(data->az, 1);
  
  display.print("Alt:");
  display.print(data->altitude, 1);
  display.print(" T:");
  display.println(data->temperature, 1);
  
  display.print("Time:");
  display.println(data->timestamp / 1000);
  
  display.display();
}

bool detectMovement(float ax, float ay, float az) {
  if (sleep_state.first_read) {
    sleep_state.last_ax = ax;
    sleep_state.last_ay = ay;
    sleep_state.last_az = az;
    sleep_state.first_read = false;
    sleep_state.stable_count = 0;
    return true;
  }
  
  float delta_ax = abs(ax - sleep_state.last_ax);
  float delta_ay = abs(ay - sleep_state.last_ay);
  float delta_az = abs(az - sleep_state.last_az);
  
  sleep_state.last_ax = ax;
  sleep_state.last_ay = ay;
  sleep_state.last_az = az;
  
  bool movement = (delta_ax > MOVEMENT_THRESHOLD || 
                   delta_ay > MOVEMENT_THRESHOLD || 
                   delta_az > MOVEMENT_THRESHOLD);
  
  if (movement) {
    sleep_state.stable_count = 0;
    return true;
  } else {
    sleep_state.stable_count++;
    return (sleep_state.stable_count < 50);
  }
}

void enterLightSleep(void) {
  Serial.println("Entering sleep mode...");
  
  if (display_ok) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("No Movement");
    display.println("Detected");
    display.println("");
    display.println("Entering Sleep");
    display.print("Wake in ");
    display.print(SLEEP_DURATION_SEC);
    display.println("s");
    display.display();
    delay(1000);  // Show message for 1 second
  }
  
  // Flush and close file before sleep
  if (logFile) {
    Serial.println("Flushing and closing log file...");
    logFile.flush();
    logFile.close();
  }
  
  // Configure timer wake source
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_SEC * 1000000ULL);
  
  Serial.println("Entering light sleep now...");
  // Enter light sleep
  esp_light_sleep_start();
  
  // ===== WAKEUP - Code continues here =====
  Serial.println("Woke up from sleep!");
  
  // Reinitialize I2C bus
  Wire.begin();
  delay(100);
  
  // Reopen log file in append mode
  Serial.println("Reopening log file...");
  logFile = SD.open("/imu_log.csv", FILE_APPEND);
  if (!logFile) {
    Serial.println("ERROR: Failed to reopen log file!");
  } else {
    Serial.println("Log file reopened successfully");
  }
  
  // Reset sleep state
  sleep_state.first_read = true;
  sleep_state.stable_count = 0;
  
  if (display_ok) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Woke from sleep");
    display.println("Resuming logging...");
    display.display();
    delay(1000);
  }
  
  Serial.println("Resuming normal operation\n");
}

void flushLogFile(void) {
  static unsigned long lastFlush = 0;
  unsigned long currentTime = millis();
  
  if (currentTime >= lastFlush && 
      (currentTime - lastFlush) > FLUSH_INTERVAL_MS) {
    logFile.flush();
    lastFlush = currentTime;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);  // Give time for Serial Monitor to open
  
  Serial.println("\n\n=== STARTING SETUP ===");
  
  Wire.begin();
  delay(500);
  
  spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  
  if (!initDisplay()) {
    Serial.println("FATAL: Display failed");
    while(1) delay(1000);
  }
  display_ok = true;
  
  if (!initSD()) {
    Serial.println("FATAL: SD failed");
    while(1) delay(1000);
  }
  
  if (!initIMU()) {
    Serial.println("FATAL: IMU failed");
    while(1) delay(1000);
  }
  icm_ok = true;
  
  if (!initBarometer()) {
    Serial.println("WARNING: Barometer failed - continuing anyway");
    mpl_ok = false;
  } else {
    mpl_ok = true;
  }
  
  if (!initLogFile()) {
    Serial.println("FATAL: Log file failed");
    while(1) delay(1000);
  }
  
  if (display_ok) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("System Ready");
    display.println("Logging...");
    display.display();
  }
  
  Serial.println("=== SETUP COMPLETE ===");
  Serial.println("Entering loop...\n");
  delay(1000);
}

void loop() {
  static int displayCounter = 0;
  
  loop_count++;
  
  // Reduced serial output - only every 50 loops
  if (loop_count % 50 == 0) {
    Serial.print("Loop ");
    Serial.println(loop_count);
  }
  
  SensorData data = {0};
  data.timestamp = millis();
  
  // Read IMU
  bool imu_success = readIMUSensors(&data);
  if (!imu_success) {
    if (loop_count % 50 == 0) {
      Serial.println("IMU read failed!");
    }
    delay(LOOP_DELAY_MS);
    return;
  }
  
  // Set default barometer values (barometer disabled for speed)
  data.altitude = 0.0f;
  data.temperature = 25.0f;
  
  // Optional: Try barometer every 50 loops (1 second) if you want altitude/temp
  if (mpl_ok && (loop_count % 50 == 0)) {
    unsigned long baro_start = millis();
    float alt = baro.getAltitude();
    unsigned long baro_time = millis() - baro_start;
    
    if (baro_time < 100) {
      data.altitude = alt;
      data.temperature = baro.getTemperature();
    } else {
      Serial.println("Barometer too slow - disabling");
      mpl_ok = false;
    }
  }
  
  // Check for movement
  bool movement_detected = detectMovement(data.ax, data.ay, data.az);
  
  if (!movement_detected) {
    Serial.println("No movement detected - entering sleep");
    enterLightSleep();
    loop_count = 0;  // Reset counter after sleep
    displayCounter = 0;
    return;
  }
  
  // Write to log
  if (!writeLogData(&data)) {
    if (loop_count % 50 == 0) {
      Serial.println("Write failed!");
    }
    delay(LOOP_DELAY_MS);
    return;
  }
  
  // Update display every 25 loops (500ms) to avoid I2C conflicts
  displayCounter++;
  if (display_ok && displayCounter >= 25) {
    display.clearDisplay();
    display.setCursor(0, 0);
    
    display.print("LOGGING [");
    display.print(loop_count);
    display.println("]");
    
    // Display accelerometer data
    display.print("A:");
    display.print(data.ax, 1);
    display.print(",");
    display.print(data.ay, 1);
    display.print(",");
    display.println(data.az, 1);
    
    // Display gyroscope data
    display.print("G:");
    display.print(data.gx, 0);
    display.print(",");
    display.print(data.gy, 0);
    display.print(",");
    display.println(data.gz, 0);
    
    // Display altitude and temperature if available
    if (mpl_ok) {
      display.print("Alt:");
      display.print(data.altitude, 1);
      display.print(" T:");
      display.println(data.temperature, 1);
    }
    
    // Display timestamp
    display.print("Time:");
    display.print(data.timestamp / 1000);
    display.println("s");
    
    display.display();
    displayCounter = 0;
  }
  
  // Flush periodically
  flushLogFile();
  
  delay(LOOP_DELAY_MS);
}
