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
#define LOOP_DELAY_MS 100

// Sleep configuration
#define SLEEP_DURATION_SEC 5
#define MOVEMENT_THRESHOLD 10.0f  // m/s² acceleration threshold

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
} sleep_state = {0.0f, 0.0f, 0.0f, true};

// Global objects (minimized where possible)
SPIClass spi(VSPI);
ICM_20948_I2C myICM;
Adafruit_MPL3115A2 baro;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
File logFile;

bool icm_ok = false;
bool mpl_ok = false;
bool display_ok = false;
int error_state = ERROR_NONE;

// Rule 5: Assertion for recovery
bool assertRange(float value, float min, float max, int errorCode) {
  if (value < min || value > max) {
    error_state = errorCode;
    return false;
  }
  return true;
}

// Rule 5: Assertion for file operations
bool assertFileValid(void) {
  if (!logFile) {
    error_state = ERROR_WRITE_FAILED;
    return false;
  }
  return true;
}

// Rule 2 & 4: Bounded display initialization
bool initDisplay(void) {
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Display Ready");
      display.display();
      return true;
    }
    delay(100);
  }
  error_state = ERROR_DISPLAY_INIT;
  return false;
}

// Rule 2 & 4: Bounded initialization with fixed retry limit
bool initSD(void) {
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (SD.begin(SD_CS, spi)) {
      return true;
    }
    delay(100);
  }
  error_state = ERROR_SD_INIT;
  return false;
}

// Rule 2 & 4: Bounded IMU initialization
bool initIMU(void) {
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    myICM.begin(Wire, 0x68);
    if (myICM.status == ICM_20948_Stat_Ok) {
      return true;
    }
    
    myICM.begin(Wire, 0x69);
    if (myICM.status == ICM_20948_Stat_Ok) {
      return true;
    }
    
    delay(100);
  }
  error_state = ERROR_IMU_INIT;
  return false;
}

// Rule 2 & 4: Bounded barometer initialization
bool initBarometer(void) {
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (baro.begin()) {
      baro.setSeaPressure(1013.26);
      return true;
    }
    delay(100);
  }
  error_state = ERROR_BARO_INIT;
  return false;
}

// Rule 2 & 4: Bounded file opening
bool initLogFile(void) {
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    logFile = SD.open("/imu_log.csv", FILE_WRITE);
    if (logFile) {
      size_t written = logFile.println("timestamp_ms,ax,ay,az,gx,gy,gz,mx,my,mz,alt_m,temp_C");
      if (written > 0) {
        logFile.flush();
        return true;
      }
      logFile.close();
    }
    delay(100);
  }
  error_state = ERROR_FILE_OPEN;
  return false;
}

// Rule 4 & 7: Read IMU sensors with validation
bool readIMUSensors(SensorData* data) {
  // Rule 5: Parameter validation
  if (data == NULL) {
    error_state = ERROR_SENSOR_RANGE;
    return false;
  }
  
  if (!icm_ok) {
    return false;
  }
  
  // Rule 7: Check return value
  if (!myICM.getAGMT()) {
    return false;
  }
  
  // Apply calibration
  data->ax = (myICM.accX() - accel_cal.offset_x) * accel_cal.scale_x;
  data->ay = (myICM.accY() - accel_cal.offset_y) * accel_cal.scale_y;
  data->az = (myICM.accZ() - accel_cal.offset_z) * accel_cal.scale_z;
  
  data->gx = myICM.gyrX();
  data->gy = myICM.gyrY();
  data->gz = myICM.gyrZ();
  data->mx = myICM.magX();
  data->my = myICM.magY();
  data->mz = myICM.magZ();
  
  // Rule 5: Validate sensor ranges
  if (!assertRange(data->ax, ACCEL_MIN, ACCEL_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->ay, ACCEL_MIN, ACCEL_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->az, ACCEL_MIN, ACCEL_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->gx, GYRO_MIN, GYRO_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->gy, GYRO_MIN, GYRO_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->gz, GYRO_MIN, GYRO_MAX, ERROR_SENSOR_RANGE)) return false;
  
  return true;
}

// Rule 4: Read barometer sensors
bool readBarometer(SensorData* data) {
  if (data == NULL || !mpl_ok) {
    return false;
  }
  
  data->altitude = baro.getAltitude();
  data->temperature = baro.getTemperature();
  
  // Rule 5: Validate ranges
  if (!assertRange(data->altitude, ALT_MIN, ALT_MAX, ERROR_SENSOR_RANGE)) return false;
  if (!assertRange(data->temperature, TEMP_MIN, TEMP_MAX, ERROR_SENSOR_RANGE)) return false;
  
  return true;
}

// Rule 4 & 7: Write data to log file with validation
bool writeLogData(const SensorData* data) {
  if (data == NULL) {
    return false;
  }
  
  // Rule 5: Verify file is valid
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
  
  // Rule 7: Check snprintf didn't overflow
  if (len < 0 || len >= (int)sizeof(line)) {
    error_state = ERROR_WRITE_FAILED;
    return false;
  }
  
  // Rule 7: Check write succeeded
  size_t written = logFile.println(line);
  if (written == 0) {
    error_state = ERROR_WRITE_FAILED;
    return false;
  }
  
  return true;
}

// Rule 4: Update display with sensor data
void updateDisplay(const SensorData* data) {
  if (!display_ok || data == NULL) {
    return;
  }
  
  display.clearDisplay();
  display.setCursor(0, 0);
  
  // Display accelerometer data
  display.print("A:");
  display.print(data->ax, 1);
  display.print(" ");
  display.print(data->ay, 1);
  display.print(" ");
  display.println(data->az, 1);
  
  // Display gyroscope data
  display.print("G:");
  display.print(data->gx, 0);
  display.print(" ");
  display.print(data->gy, 0);
  display.print(" ");
  display.println(data->gz, 0);
  
  // Display altitude and temperature
  display.print("Alt:");
  display.print(data->altitude, 1);
  display.println("m");
  
  display.print("Temp:");
  display.print(data->temperature, 1);
  display.println("C");
  
  // Display timestamp
  display.print("Time:");
  display.println(data->timestamp / 1000);
  
  display.display();
}

// Rule 4: Check for movement based on acceleration threshold
bool detectMovement(float ax, float ay, float az) {
  if (sleep_state.first_read) {
    sleep_state.last_ax = ax;
    sleep_state.last_ay = ay;
    sleep_state.last_az = az;
    sleep_state.first_read = false;
    return true;  // Assume movement on first read
  }
  
  // Rule 6: Local scope for delta calculations
  float delta_ax = abs(ax - sleep_state.last_ax);
  float delta_ay = abs(ay - sleep_state.last_ay);
  float delta_az = abs(az - sleep_state.last_az);
  
  // Update last values
  sleep_state.last_ax = ax;
  sleep_state.last_ay = ay;
  sleep_state.last_az = az;
  
  // Check if any axis exceeds threshold
  return (delta_ax > MOVEMENT_THRESHOLD || 
          delta_ay > MOVEMENT_THRESHOLD || 
          delta_az > MOVEMENT_THRESHOLD);
}

// Rule 4: Enter light sleep mode
void enterLightSleep(void) {
  if (display_ok) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Entering Sleep");
    display.print("Wake in ");
    display.print(SLEEP_DURATION_SEC);
    display.println("s");
    display.display();
  }
  
  // Flush and close file before sleep
  if (logFile) {
    logFile.flush();
    logFile.close();
  }
  
  // Configure timer wake source
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_SEC * 1000000ULL);
  
  // Enter light sleep
  esp_light_sleep_start();
  
  // After wake, reinitialize file
  if (!logFile) {
    logFile = SD.open("/imu_log.csv", FILE_APPEND);
  }
}

// Rule 4: Periodic flush with bounded check
void flushLogFile(void) {
  static unsigned long lastFlush = 0;
  
  // Rule 6: Local scope for current time
  unsigned long currentTime = millis();
  
  // Rule 5: Check time is monotonic
  if (currentTime >= lastFlush && 
      (currentTime - lastFlush) > FLUSH_INTERVAL_MS) {
    logFile.flush();
    lastFlush = currentTime;
  }
}

void setup() {
  Wire.begin();
  delay(500);
  
  spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  
  // Rule 7: Check all initialization return values
  if (!initDisplay()) return;
  display_ok = true;
  
  if (!initSD()) return;
  if (!initIMU()) return;
  
  icm_ok = true;
  
  if (!initBarometer()) return;
  
  mpl_ok = true;
  
  if (!initLogFile()) return;
  
  // Initial sleep cycle
  delay(100);
  enterLightSleep();
}

// Rule 4: Main loop kept under 60 lines
void loop() {
  // Rule 6: Local scope for sensor data
  SensorData data = {0};
  
  data.timestamp = millis();
  
  // Read sensors with error checking
  bool imu_success = readIMUSensors(&data);
  bool baro_success = readBarometer(&data);
  
  // Rule 5: Only process if we have valid data
  if (!imu_success || !baro_success) {
    delay(LOOP_DELAY_MS);
    return;
  }
  
  // Check for movement
  bool movement_detected = detectMovement(data.ax, data.ay, data.az);
  
  if (!movement_detected) {
    // No movement, enter sleep mode
    enterLightSleep();
    return;
  }
  
  // Movement detected, log data
  if (!writeLogData(&data)) {
    delay(LOOP_DELAY_MS);
    return;
  }
  
  // Update display
  updateDisplay(&data);
  
  flushLogFile();
  
  delay(LOOP_DELAY_MS);
}
