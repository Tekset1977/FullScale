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
#define MOVEMENT_THRESHOLD 200.0f  // m/s² acceleration threshold
#define MIN_AWAKE_MS 5000 

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
static unsigned long last_movement_ms = 0;

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
    if (!resetICM(0x68)) {
      Serial.println("resetICM failed");
      delay(100);
      continue;
    }
    myICM.begin(Wire, 0x68);
    if (myICM.status != ICM_20948_Stat_Ok) {
      Serial.printf("myICM.begin failed, status=%d\n", myICM.status);
      delay(100);
      continue;
    }
    myICM.startupMagnetometer();
    delay(100);
    return true;
  }
  error_state = ERROR_IMU_INIT;
  return false;
}

// Rule 2 & 4: Bounded barometer initialization
bool initBarometer(void) {
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    if (baro.begin()) {

      baro.setSeaPressure(1013.26);

      // Set to OSR = 0 (fastest mode)
      Wire.beginTransmission(0x60);
      Wire.write(0x26);
      Wire.write(0x39);
      Wire.endTransmission();

      delay(20);

      // Verify control register
      Wire.beginTransmission(0x60);
      Wire.write(0x26);
      Wire.endTransmission(false);
      Wire.requestFrom(0x60, 1);

      if (Wire.available()) {
        uint8_t ctrl_reg = Wire.read();
        // Optional: could validate ctrl_reg here
      }

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
  
  // Rule 7: Get sensor data and check status
  myICM.getAGMT();
 if (myICM.status != ICM_20948_Stat_Ok && 
    myICM.status != ICM_20948_Stat_WrongID) { // WrongID often means just mag timeout
    Serial.printf("getAGMT status: %d\n", myICM.status);
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

  data->altitude    = 0.0f;
  data->temperature = 25.0f;

  unsigned long start = millis();
  data->altitude = baro.getAltitude();
  if (millis() - start > 200) {   // raise from 50ms to 200ms
    Serial.println("BARO altitude timeout");
    return false;
  }

  start = millis();
  data->temperature = baro.getTemperature();
  if (millis() - start > 200) {
    Serial.println("BARO temp timeout");
    return false;
  }

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
void updateDisplay(const SensorData* data, bool moving, unsigned long idle_ms) {
  if (!display_ok || data == NULL) return;

  display.clearDisplay();
  display.setCursor(0, 0);

  // Accelerometer
  display.print("A:");
  display.print(data->ax, 0);
  display.print(" ");
  display.print(data->ay, 0);
  display.print(" ");
  display.println(data->az, 0);

  // Gyroscope
  display.print("G:");
  display.print(data->gx, 0);
  display.print(" ");
  display.print(data->gy, 0);
  display.print(" ");
  display.println(data->gz, 0);

  // Altitude and temperature on one line
  display.print("Alt:");
  display.print(data->altitude, 1);
  display.print("m  T:");
  display.print(data->temperature, 1);
  display.println("C");

  // Timestamp
  display.print("Time: ");
  display.print(data->timestamp / 1000);
  display.println("s");

  // Movement status
  display.print("MOV: ");
  display.println(moving ? "YES" : "NO ");

  // Sleep countdown
  long remaining = ((long)MIN_AWAKE_MS - (long)idle_ms) / 1000;
  if (remaining < 0) remaining = 0;
  display.print("Sleep in: ");
  display.print(remaining);
  display.println("s");

  display.display();
}

// Rule 4: Check for movement based on acceleration threshold
bool detectMovement(float ax, float ay, float az) {
  if (sleep_state.first_read) {
    sleep_state.last_ax = ax;
    sleep_state.last_ay = ay;
    sleep_state.last_az = az;
    sleep_state.first_read = false;
    return true;
  }

  float delta_ax = abs(ax - sleep_state.last_ax);
  float delta_ay = abs(ay - sleep_state.last_ay);
  float delta_az = abs(az - sleep_state.last_az);

  sleep_state.last_ax = ax;
  sleep_state.last_ay = ay;
  sleep_state.last_az = az;

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
    display.display();
  }

  if (logFile) { logFile.flush(); logFile.close(); }

  esp_sleep_enable_timer_wakeup(SLEEP_DURATION_SEC * 1000000ULL);
  Serial.println("lalelulelo");
  Serial.flush();               // ensure output completes before sleep
  esp_light_sleep_start();
  Serial.println("wakey-wakey");

  resetI2CBus(); 

  icm_ok = initIMU();
  if (icm_ok) {
  // Re-enable magnetometer aux I2C
  myICM.startupMagnetometer();
  delay(100);
  }
  Serial.printf("IMU: %s\n", icm_ok ? "OK" : "FAIL");

  mpl_ok = initBarometer();
  Serial.printf("BARO: %s\n", mpl_ok ? "OK" : "FAIL");
  if (mpl_ok) delay(150);

  display_ok = initDisplay();
  Serial.printf("DISP: %s\n", display_ok ? "OK" : "FAIL");

  if (!SD.begin(SD_CS, spi)) {
  Serial.println("SD re-init FAIL");
  } else {
  Serial.println("SD re-init OK");
  logFile = SD.open("/imu_log.csv", FILE_APPEND);
  if (!logFile) {
    Serial.println("File open FAIL");
  } else {
    Serial.println("File open OK");
  }
  }
}

// Rule 4: Periodic flush with bounded check
void flushLogFile(void) {
  static unsigned long lastFlush = 0;
  if (!logFile) return;
  
  // Rule 6: Local scope for current time
  unsigned long currentTime = millis();
  
  // Rule 5: Check time is monotonic
  if (currentTime >= lastFlush && 
      (currentTime - lastFlush) > FLUSH_INTERVAL_MS) {
    logFile.flush();
    lastFlush = currentTime;
  }
}

void resetI2CBus(void) {
  Wire.end();
  delay(50);

  pinMode(21, OUTPUT);  // SDA
  pinMode(22, OUTPUT);  // SCL
  digitalWrite(21, HIGH);
  digitalWrite(22, HIGH);
  delay(10);

  // 18 pulses instead of 9 — handles deeper stuck states
  for (int i = 0; i < 18; i++) {
    digitalWrite(22, HIGH); delayMicroseconds(50);
    digitalWrite(22, LOW);  delayMicroseconds(50);
  }
  // STOP condition
  digitalWrite(21, LOW);  delayMicroseconds(50);
  digitalWrite(22, HIGH); delayMicroseconds(50);
  digitalWrite(21, HIGH); delayMicroseconds(50);

  delay(50);
  pinMode(21, INPUT_PULLUP);
  pinMode(22, INPUT_PULLUP);
  delay(10);

  // Check SDA is actually high before handing to Wire
  if (digitalRead(21) == LOW) {
    Serial.println("SDA still stuck low after recovery!");
  }
  if (digitalRead(22) == LOW) {
    Serial.println("SCL still stuck low after recovery!");
  }

  Wire.begin(21, 22);
  Wire.setTimeOut(100);
  delay(100);
}

bool resetICM(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x06);
  Wire.write(0x80);
  uint8_t err = Wire.endTransmission();
  Serial.printf("resetICM addr=0x%02X endTransmission err=%d\n", addr, err);
  if (err != 0) return false;

  delay(250);  // chip needs ~100ms to reset, 250 to be safe

  // Confirm reset completed — chip auto-clears the bit when done
  Wire.beginTransmission(addr);
  Wire.write(0x06);
  Wire.endTransmission(false);
  Wire.requestFrom(addr, (uint8_t)1);
  if (!Wire.available()) return false;
  
  uint8_t val = Wire.read();
  return (val & 0x80) == 0;  // reset done when bit clears
}

void setup() {
  Wire.begin(21, 22);
  Wire.setTimeOut(100);
  delay(500);
  Serial.begin(115200);
  
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
  last_movement_ms = millis();
  enterLightSleep();
}

// Rule 4: Main loop kept under 60 lines
void loop() {
  // Rule 6: Local scope for sensor data
  Serial.println("loop tick");
  SensorData data = {0};
  data.timestamp = millis();
  Serial.println("A");
  // Read sensors with error checking
  bool imu_success = readIMUSensors(&data);
  bool baro_success = readBarometer(&data);
  Serial.println("B");
  // Rule 5: Only process if we have valid data
  if (!imu_success || !baro_success) {
    if (display_ok) {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Sensor fail: ");
    display.print(imu_success  ? "" : "IMU ");
    display.print(baro_success ? "" : "BARO");
    display.display();
  }
  Serial.printf("Sensor fail — IMU:%d BARO:%d err:%d\n",
                imu_success, baro_success, error_state);
  delay(LOOP_DELAY_MS);
  return;
  }
  Serial.println("sensors OK");
  
  // Check for movement
  bool movement_detected = detectMovement(data.ax, data.ay, data.az);
  
  if (movement_detected) {
    last_movement_ms = millis();  // Reset the idle timer on any movement
  }
  unsigned long idle_ms = millis() - last_movement_ms;

 Serial.println("writing log");
  writeLogData(&data);

  Serial.println("updating display");
  updateDisplay(&data, movement_detected, idle_ms);

  Serial.println("flushing");
  flushLogFile();

  if (idle_ms > MIN_AWAKE_MS) enterLightSleep();
  delay(LOOP_DELAY_MS);
}
