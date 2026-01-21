#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <Adafruit_INA3221.h>
#include <ModbusMaster.h>
#include <esp_sleep.h>

// ==================== PIN DEFINITIONS ====================
#define SD_CS     5
#define SPI_SCK   18
#define SPI_MISO  19
#define SPI_MOSI  23
#define WAKE_PIN  15

// RS485/Modbus pins
#define RX2_PIN    16   // MAX485 RO to ESP32 GPIO16
#define TX2_PIN    17   // MAX485 DI from ESP32 GPIO17
#define RE_DE_PIN  4    // MAX485 DE & RE tied to ESP32 GPIO4

// ==================== CONFIGURATION ====================
#define ALT_CHANGE_THRESHOLD 1.0    // meters
#define MOTION_THRESHOLD 0.15       // g's for accelerometer
#define STABLE_TIME 3000            // ms to wait before taking soil reading
#define MODBUS_BAUD 4800
#define SLAVE_ID 1

// ==================== GLOBAL OBJECTS ====================
SPIClass spi(VSPI);
ICM_20948_I2C myICM;
Adafruit_INA3221 ina3221;
ModbusMaster soilSensor;
File logFile;

bool icm_ok = false;
bool mpl_ok = false;
bool ina_ok = false;
bool modbus_ok = false;

// Motion detection variables
float last_altitude = 0;
unsigned long last_motion_time = 0;
bool first_reading = true;

// Accelerometer baseline for motion detection
float baseline_ax = 0, baseline_ay = 0, baseline_az = 0;
bool baseline_set = false;

// ==================== RS485 DIRECTION CONTROL ====================
void preTransmission() {
  digitalWrite(RE_DE_PIN, HIGH);
  delayMicroseconds(400);
}

void postTransmission() {
  delayMicroseconds(400);
  digitalWrite(RE_DE_PIN, LOW);
}

// ==================== MPL3115A2 FUNCTIONS ====================
bool mplReadStatusNonBlocking(uint8_t &status_out) {
  Wire.beginTransmission(0x60);
  Wire.write(0x00);
  uint8_t err = Wire.endTransmission(false);
  if (err != 0) return false;

  delayMicroseconds(100);
  
  int n = Wire.requestFrom(0x60, (uint8_t)1);
  if (n < 1) return false;

  status_out = Wire.read();
  return true;
}

bool mplVerifyWhoAmI() {
  Wire.beginTransmission(0x60);
  Wire.write(0x0C);
  if (Wire.endTransmission(false) != 0) return false;
  
  if (Wire.requestFrom(0x60, (uint8_t)1) < 1) return false;
  
  uint8_t whoami = Wire.read();
  Serial.print("[DEBUG] MPL WHO_AM_I: 0x");
  Serial.println(whoami, HEX);
  return (whoami == 0xC4);
}

bool mpl3115_reset_and_init() {
  Serial.println("[RESET] Starting MPL3115A2 software reset...");
  
  if (!mplVerifyWhoAmI()) {
    Serial.println("[ERROR] MPL3115A2 not responding to WHO_AM_I!");
    return false;
  }

  Wire.setTimeOut(1000);
  
  // Enter standby mode
  Wire.beginTransmission(0x60);
  Wire.write(0x26);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(20);

  Serial.println("[RESET] MPL3115A2 ready for reconfiguration");
  return true;
}

void configureMPLInterrupt() {
  Serial.println("[CONFIG] Configuring MPL3115A2...");
  
  Wire.beginTransmission(0x60);
  Wire.write(0x26);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(20);

  // Calculate sea-level pressure from local conditions
  float local_pressure_pa = 1016.49 * 100.0; // Convert mbar to Pa
  float sea_level_pressure_pa = local_pressure_pa * pow(1.0 - (194 / 44330.0), -5.255);
  float sea_level_pressure_mbar = sea_level_pressure_pa / 100.0;
  
  Serial.print("[CONFIG] Calculated sea-level pressure: ");
  Serial.print(sea_level_pressure_mbar, 2);
  Serial.println(" mbar");
  
  // Set BAR_IN registers
  uint16_t bar_in = (uint16_t)(sea_level_pressure_pa / 2.0);
  
  Wire.beginTransmission(0x60);
  Wire.write(0x14);
  Wire.write((bar_in >> 8) & 0xFF);
  Wire.endTransmission();
  delay(5);
  
  Wire.beginTransmission(0x60);
  Wire.write(0x15);
  Wire.write(bar_in & 0xFF);
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(0x60);
  Wire.write(0x13);
  Wire.write(0x07);
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(0x60);
  Wire.write(0x29);
  Wire.write(0x80);
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(0x60);
  Wire.write(0x2A);
  Wire.write(0x80);
  Wire.endTransmission();
  delay(5);

  Wire.beginTransmission(0x60);
  Wire.write(0x26);
  Wire.write(0xB9);
  Wire.endTransmission();
  
  Serial.println("[CONFIG] Set to ACTIVE altimeter mode (CONTINUOUS)");
  delay(100);
}

float safeReadAltitude(unsigned long timeout_ms = 2000) {
  unsigned long start = millis();
  uint8_t s;

  while (millis() - start < timeout_ms) {
    if (mplReadStatusNonBlocking(s)) {
      if (s & 0x08) {
        Wire.beginTransmission(0x60);
        Wire.write(0x01);
        if (Wire.endTransmission(false) == 0) {
          if (Wire.requestFrom(0x60, (uint8_t)3) >= 3) {
            uint8_t msb = Wire.read();
            uint8_t csb = Wire.read();
            uint8_t lsb = Wire.read();
            
            int32_t alt_raw = ((uint32_t)msb << 24) | ((uint32_t)csb << 16) | ((uint32_t)lsb << 8);
            alt_raw >>= 8;
            
            float altitude = (float)alt_raw / 256.0f;
            return altitude;
          }
        }
      }
    }
    delay(10);
  }

  Serial.println("[ERROR] safeReadAltitude timeout");
  return NAN;
}

float safeReadTemperature(unsigned long timeout_ms = 800) {
  unsigned long start = millis();
  uint8_t s;

  while (millis() - start < timeout_ms) {
    if (mplReadStatusNonBlocking(s)) {
      if (s & 0x02) {
        Wire.beginTransmission(0x60);
        Wire.write(0x04);
        if (Wire.endTransmission(false) == 0) {
          if (Wire.requestFrom(0x60, (uint8_t)2) >= 2) {
            uint8_t msb = Wire.read();
            uint8_t lsb = Wire.read();
            
            int16_t temp_raw = ((int16_t)msb << 8) | lsb;
            temp_raw >>= 4;
            
            float temperature = (float)temp_raw / 16.0f;
            return temperature;
          }
        }
      }
    }
    delay(10);
  }

  return NAN;
}

// ==================== SOIL SENSOR FUNCTIONS ====================
struct SoilData {
  float ec_uScm;
  float ph;
  float nitrates_mgkg;
  bool valid;
};

SoilData readSoilSensor() {
  SoilData data = {0, 0, 0, false};
  
  if (!modbus_ok) {
    Serial.println("[SOIL] Modbus not initialized");
    return data;
  }

  Serial.println("[SOIL] Reading EC, pH, Nitrates...");
  
  // Read 3 registers starting at 0x0002
  uint8_t result = soilSensor.readHoldingRegisters(0x0002, 3);
  
  if (result == soilSensor.ku8MBSuccess) {
    uint16_t ec_raw = soilSensor.getResponseBuffer(0);  // reg 0x0002
    uint16_t ph_raw = soilSensor.getResponseBuffer(1);  // reg 0x0003
    uint16_t n_raw  = soilSensor.getResponseBuffer(2);  // reg 0x0004
    
    data.ec_uScm = (float)ec_raw; 
    data.ph = ph_raw * 0.1f;
    data.nitrates_mgkg = (float)n_raw;
    data.valid = true;
    
    Serial.print("[SOIL] EC: "); Serial.print(data.ec_uScm, 0); Serial.println(" uS/cm");
    Serial.print("[SOIL] pH: "); Serial.println(data.ph, 1);
    Serial.print("[SOIL]  N: "); Serial.print(data.nitrates_mgkg, 0); Serial.println(" mg/kg");
  } else {
    Serial.print("[SOIL] Read FAILED - Error: 0x");
    Serial.println(result, HEX);
  }
  
  return data;
}

// ==================== MOTION DETECTION ====================
bool detectMotion(float ax, float ay, float az) {
  if (!baseline_set) {
    // Set initial baseline
    baseline_ax = ax;
    baseline_ay = ay;
    baseline_az = az;
    baseline_set = true;
    last_motion_time = millis();
    return true; // Consider first reading as motion
  }
  
  // Calculate delta from baseline
  float delta_x = fabs(ax - baseline_ax);
  float delta_y = fabs(ay - baseline_ay);
  float delta_z = fabs(az - baseline_az);
  
  // Check if any axis exceeds threshold
  if (delta_x > MOTION_THRESHOLD || 
      delta_y > MOTION_THRESHOLD || 
      delta_z > MOTION_THRESHOLD) {
    
    Serial.printf("[MOTION] Detected: dX=%.3f dY=%.3f dZ=%.3f\n", delta_x, delta_y, delta_z);
    last_motion_time = millis();
    
    // Update baseline to current values
    baseline_ax = ax;
    baseline_ay = ay;
    baseline_az = az;
    
    return true;
  }
  
  return false;
}

bool isStable() {
  unsigned long stable_duration = millis() - last_motion_time;
  
  if (stable_duration >= STABLE_TIME) {
    Serial.printf("[STABLE] No motion for %.1f seconds\n", stable_duration / 1000.0f);
    return true;
  }
  
  return false;
}

// ==================== I2C BUS CLEARING ====================
void clearI2CBus() {
  Serial.println("[I2C] Clearing I2C bus...");
  
  const int SDA_PIN = 21;
  const int SCL_PIN = 22;
  
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
  }
  
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(5);
  
  Serial.println("[I2C] Bus cleared");
}

// ==================== SLEEP MODE ====================
void enterLightSleep() {
  Serial.println("\n>>> [SLEEP] Entering light sleep mode...");
  Serial.println(">>> [SLEEP] Will wake every 30 seconds to check for motion...");
  Serial.flush();

  // Close file before sleep
  if (logFile) {
    logFile.close();
  }

  esp_sleep_enable_timer_wakeup(30 * 1000000); // 30 seconds
  esp_light_sleep_start();

  Serial.println("\n*** [WAKE] Timer wake event ***");
  Serial.println("*** [WAKE] Reinitializing peripherals...");

  // Reinitialize I2C
  clearI2CBus();
  delay(50);
  Wire.end();
  delay(10);
  Wire.begin();
  Wire.setClock(100000);
  delay(100);
  
  // Reinitialize ICM20948
  if (icm_ok) {
    myICM.begin(Wire, 0x68);
    if (myICM.status != ICM_20948_Stat_Ok) {
      myICM.begin(Wire, 0x69);
    }
    Serial.println("*** [WAKE] ICM20948 reinitialized");
  }

  // Reinitialize MPL3115A2
  if (mpl3115_reset_and_init()) {
    configureMPLInterrupt();
    delay(200);
    Serial.println("*** [WAKE] MPL3115A2 reinitialized");
    mpl_ok = true;
  } else {
    Serial.println("*** [WAKE] MPL3115A2 reinit failed");
    mpl_ok = false;
  }

  // Reinitialize SPI for SD card
  spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  Serial.println("*** [WAKE] SPI reinitialized");

  // Reinitialize Modbus
  Serial2.begin(MODBUS_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);
  Serial.println("*** [WAKE] Modbus reinitialized");

  // Reopen log file
  logFile = SD.open("/sensor_log.csv", FILE_APPEND);
  if (!logFile) {
    Serial.println("*** [WAKE] Failed to reopen log file!");
  }

  // Reset motion detection
  baseline_set = false;
  first_reading = true;

  Serial.println("*** [WAKE] Active monitoring resumed ***\n");
}

// ==================== SETUP ====================
void setup() {
  Wire.begin();
  Wire.setClock(100000);
  Serial.begin(115200);
  delay(300);

  Serial.println("\n========================================");
  Serial.println("CUm Chalice - IMU Motion + Soil Sensor");
  Serial.println("========================================");

  // Initialize RS485 control pin
  pinMode(RE_DE_PIN, OUTPUT);
  digitalWrite(RE_DE_PIN, LOW); // Receive mode by default

  // Initialize SPI for SD card
  Serial.println("[INIT] Initializing SPI bus...");
  spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  // Initialize SD card
  Serial.println("[INIT] Initializing SD card...");
  if (!SD.begin(SD_CS, spi)) {
    Serial.println("[ERROR] SD init failed!");
    while (1);
  }
  Serial.println("[OK] SD initialized.");

  // Initialize ICM20948
  Serial.println("[INIT] Initializing ICM20948...");
  myICM.begin(Wire, 0x68);
  if (myICM.status != ICM_20948_Stat_Ok) {
    myICM.begin(Wire, 0x69);
  }
  if (myICM.status == ICM_20948_Stat_Ok) {
    icm_ok = true;
    Serial.println("[OK] ICM20948 ready.");
  } else {
    Serial.println("[ERROR] ICM20948 init failed!");
  }

  // Initialize MPL3115A2
  Serial.println("[INIT] Initializing MPL3115A2...");
  if (mplVerifyWhoAmI()) {
    mpl_ok = true;
    configureMPLInterrupt();
    Serial.println("[OK] MPL3115A2 ready.");
    delay(500);
  } else {
    Serial.println("[ERROR] MPL3115A2 not detected!");
    mpl_ok = false;
  }

  // Initialize INA3221
  if (ina3221.begin()) {
    ina_ok = true;
    Serial.println("[OK] INA3221 present");
  } else {
    ina_ok = false;
    Serial.println("[WARN] INA3221 not detected");
  }

  // Initialize Modbus
  Serial.println("[INIT] Initializing Modbus (RS485)...");
  Serial2.begin(MODBUS_BAUD, SERIAL_8N1, RX2_PIN, TX2_PIN);
  soilSensor.begin(SLAVE_ID, Serial2);
  soilSensor.preTransmission(preTransmission);
  soilSensor.postTransmission(postTransmission);
  modbus_ok = true;
  Serial.println("[OK] Modbus initialized.");

  // Create/clear log file
  if (SD.exists("/sensor_log.csv")) {
    SD.remove("/sensor_log.csv");
    Serial.println("[INFO] Removed old log file");
  }

  logFile = SD.open("/sensor_log.csv", FILE_WRITE);
  if (!logFile) {
    Serial.println("[ERROR] Failed to open file for writing!");
    while (1);
  }
  logFile.println("timestamp_ms,ax,ay,az,gx,gy,gz,mx,my,mz,alt_m,temp_C,ec_uScm,ph,nitrates_mgkg,soil_valid");
  logFile.close();
  Serial.println("[OK] Header written to CSV file.");

  Serial.println("========================================");
  Serial.println("[STATUS] Setup complete!");
  Serial.println("[STATUS] Monitoring for motion...");
  Serial.println("========================================\n");
}

// ==================== MAIN LOOP ====================
void loop() {
  static unsigned long last_log = 0;
  static bool soil_reading_taken = false;
  
  unsigned long now = millis();

  // Read IMU data
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  float mx = 0, my = 0, mz = 0;

  if (icm_ok && myICM.dataReady()) {
    myICM.getAGMT();
    ax = myICM.accX();
    ay = myICM.accY();
    az = myICM.accZ();
    gx = myICM.gyrX();
    gy = myICM.gyrY();
    gz = myICM.gyrZ();
    mx = myICM.magX();
    my = myICM.magY();
    mz = myICM.magZ();
  }

  // Read altitude/temp
  float altitude = NAN;
  float temperature = NAN;
  
  if (mpl_ok) {
    altitude = safeReadAltitude(500);
    temperature = safeReadTemperature(100);
  }

  // Check for motion
  bool motion = detectMotion(ax, ay, az);
  
  if (motion) {
    soil_reading_taken = false; // Reset flag when motion detected
  }

  // Initialize soil data
  SoilData soil = {0, 0, 0, false};

  // If stable and haven't taken reading yet, read soil sensor
  if (isStable() && !soil_reading_taken) {
    Serial.println("\n[TRIGGER] System stable - reading soil sensor!");
    soil = readSoilSensor();
    soil_reading_taken = true;
    Serial.println();
  }

  // Log data every 500ms
  if (now - last_log >= 500) {
    last_log = now;
    
    char line[250];
    snprintf(line, sizeof(line),
             "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.1f,%.1f,%.1f,%d",
             now, ax, ay, az, gx, gy, gz, mx, my, mz,
             isnan(altitude) ? -9999.0f : altitude,
             isnan(temperature) ? -9999.0f : temperature,
             soil.ec_uScm, soil.ph, soil.nitrates_mgkg, soil.valid ? 1 : 0);

    logFile = SD.open("/sensor_log.csv", FILE_APPEND);
    if (logFile) {
      logFile.println(line);
      logFile.close();
      Serial.print("[LOG] ");
      Serial.println(line);
    } else {
      Serial.println("[ERROR] Failed to write to log file!");
    }
  }

  // Check if should enter sleep (stable for long time with soil reading complete)
  if (isStable() && soil_reading_taken && (millis() - last_motion_time > 10000)) {
    Serial.println("\n======================================");
    Serial.println("[SLEEP] System stable with soil reading complete");
    Serial.println("[SLEEP] Entering sleep mode...");
    Serial.println("======================================");
    
    soil_reading_taken = false;
    enterLightSleep();
  }

  delay(100);
}