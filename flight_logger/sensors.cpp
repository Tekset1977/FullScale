#include "sensors.h"
#include <Wire.h>
#include <ICM_20948.h>
#include <Adafruit_MPL3115A2.h>

// -----------------------------------------------------------------------------
//  Module-private hardware objects
// -----------------------------------------------------------------------------
static ICM_20948_I2C      g_icm;
static Adafruit_MPL3115A2 g_baro;

static const AccelCal ACCEL_CAL = {
    -40.5f, -5.0f, 16.0f,   // offsets x, y, z
     1.0f,  1.0f,  1.0f     // scales  x, y, z
};

// =============================================================================
//  I2C low-level helpers (private)
// =============================================================================

// Write one register byte; returns true on ACK.
static bool i2cWriteReg(uint8_t dev_addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(dev_addr);
    size_t wb1 = Wire.write(reg);
    size_t wb2 = Wire.write(value);
    uint8_t err = Wire.endTransmission();
    if (wb1 != 1U || wb2 != 1U) { return false; }
    if (err  != 0U)              { return false; }
    return true;
}

// Request N bytes from a register; returns bytes received.
static uint8_t i2cReadBytes(uint8_t dev_addr, uint8_t reg,
                             uint8_t* buf, uint8_t len) {
    if (!assertNotNull(buf, ERR_SENSOR_RANGE)) { return 0U; }
    if (len == 0U) { return 0U; }

    Wire.beginTransmission(dev_addr);
    size_t wb  = Wire.write(reg);
    uint8_t err = Wire.endTransmission(false);
    if (wb != 1U || err != 0U) { return 0U; }

    uint8_t received = (uint8_t)Wire.requestFrom(dev_addr, len);
    if (received != len) { return 0U; }

    for (uint8_t i = 0U; i < len; i++) {
        buf[i] = Wire.read();
    }
    return received;
}

// =============================================================================
//  IMU
// =============================================================================

static bool resetICM(uint8_t addr) {
    if (addr == 0U || addr > 127U) { return false; }

    bool ok = i2cWriteReg(addr, 0x06U, 0x80U);
    if (!ok) {
#ifdef DEBUG
        Serial.printf("resetICM addr=0x%02X write failed\n", addr);
#endif
        return false;
    }
    delay(500);
    return true;
}

bool initIMU(void) {
    if (I2C_ADDR_ICM == 0U) {
        g_error_state = ERR_IMU_INIT;
        return false;
    }

    for (int retry = 0; retry < CFG_MAX_RETRIES; retry++) {
        if (!resetICM(I2C_ADDR_ICM)) {
#ifdef DEBUG
            Serial.println("resetICM failed");
#endif
            delay(100);
            continue;
        }
        g_icm.begin(Wire, I2C_ADDR_ICM);
        if (g_icm.status != ICM_20948_Stat_Ok) {
#ifdef DEBUG
            Serial.printf("ICM begin failed, status=%d\n", g_icm.status);
#endif
            delay(100);
            continue;
        }
        delay(100);
        return true;
    }
    g_error_state = ERR_IMU_INIT;
    return false;
}

// Select ICM register bank (private)
static bool selectICMBank(uint8_t bank) {
    if (bank > 3U)          { return false; }
    if (I2C_ADDR_ICM == 0U) { return false; }
    return i2cWriteReg(I2C_ADDR_ICM, 0x7FU, (uint8_t)(bank << 4));
}

bool readIMUSensors(SensorData* data) {
    if (!assertNotNull(data, ERR_SENSOR_RANGE)) { return false; }
    if (!g_icm_ok) { return false; }

    if (!selectICMBank(0U)) { return false; }

    uint8_t raw[12] = {0U};
    if (i2cReadBytes(I2C_ADDR_ICM, 0x2DU, raw, 12U) != 12U) { return false; }

    int16_t raw_ax = (int16_t)((uint16_t)raw[0]  << 8) | raw[1];
    int16_t raw_ay = (int16_t)((uint16_t)raw[2]  << 8) | raw[3];
    int16_t raw_az = (int16_t)((uint16_t)raw[4]  << 8) | raw[5];
    int16_t raw_gx = (int16_t)((uint16_t)raw[6]  << 8) | raw[7];
    int16_t raw_gy = (int16_t)((uint16_t)raw[8]  << 8) | raw[9];
    int16_t raw_gz = (int16_t)((uint16_t)raw[10] << 8) | raw[11];

    data->ax = ((float)raw_ax / 16.384f - ACCEL_CAL.offset_x) * ACCEL_CAL.scale_x;
    data->ay = ((float)raw_ay / 16.384f - ACCEL_CAL.offset_y) * ACCEL_CAL.scale_y;
    data->az = ((float)raw_az / 16.384f - ACCEL_CAL.offset_z) * ACCEL_CAL.scale_z;
    data->gx = (float)raw_gx / 131.0f;
    data->gy = (float)raw_gy / 131.0f;
    data->gz = (float)raw_gz / 131.0f;
    data->mx = 0.0f;
    data->my = 0.0f;
    data->mz = 0.0f;

    if (!assertRange(data->ax, ACCEL_MIN_MPS2, ACCEL_MAX_MPS2, ERR_SENSOR_RANGE)) { return false; }
    if (!assertRange(data->ay, ACCEL_MIN_MPS2, ACCEL_MAX_MPS2, ERR_SENSOR_RANGE)) { return false; }
    if (!assertRange(data->az, ACCEL_MIN_MPS2, ACCEL_MAX_MPS2, ERR_SENSOR_RANGE)) { return false; }
    if (!assertRange(data->gx, GYRO_MIN_DPS,   GYRO_MAX_DPS,   ERR_SENSOR_RANGE)) { return false; }
    if (!assertRange(data->gy, GYRO_MIN_DPS,   GYRO_MAX_DPS,   ERR_SENSOR_RANGE)) { return false; }
    if (!assertRange(data->gz, GYRO_MIN_DPS,   GYRO_MAX_DPS,   ERR_SENSOR_RANGE)) { return false; }
    return true;
}

// =============================================================================
//  Barometer
// =============================================================================

static bool configureBaroRegisters(void) {
    bool ok = i2cWriteReg(I2C_ADDR_BARO, 0x26U, 0x00U);
    if (!ok) { return false; }
    delay(10);
    ok = i2cWriteReg(I2C_ADDR_BARO, 0x26U, 0x80U);
    if (!ok) { return false; }
    delay(20);
    return true;
}

bool initBarometer(void) {
    if (I2C_ADDR_BARO == 0U) {
        g_error_state = ERR_BARO_INIT;
        return false;
    }

    for (int retry = 0; retry < CFG_MAX_RETRIES; retry++) {
        if (!g_baro.begin()) { delay(100); continue; }
        g_baro.setSeaPressure(1013.26f);
        if (!configureBaroRegisters()) { delay(100); continue; }
        return true;
    }
    g_error_state = ERR_BARO_INIT;
    return false;
}

// Read baseline altitude for zeroing (called once in setup)
float baroReadAltitude(void) {
    return g_baro.getAltitude();
}

static void triggerBarometer(void) {
    if (!g_mpl_ok)          { return; }
    if (I2C_ADDR_BARO == 0U) { return; }
    bool ok = i2cWriteReg(I2C_ADDR_BARO, 0x26U, 0x82U);
#ifdef DEBUG
    if (!ok) { Serial.println("Baro trigger write failed"); }
#endif
}

static bool readBaroRawBytes(float* alt_out, float* temp_out) {
    if (!assertNotNull(alt_out,  ERR_SENSOR_RANGE)) { return false; }
    if (!assertNotNull(temp_out, ERR_SENSOR_RANGE)) { return false; }

    uint8_t buf[5] = {0U};
    if (i2cReadBytes(I2C_ADDR_BARO, 0x01U, buf, 5U) != 5U) { return false; }

    int32_t alt_raw = ((int32_t)(int8_t)buf[0] << 12) |
                      ((uint32_t)buf[1]          <<  4) |
                       (buf[2] >> 4);
    float alt  = (float)alt_raw / 16.0f;

    int16_t t_raw = ((int16_t)(int8_t)buf[3] << 8) | (int16_t)buf[4];
    float   temp  = (float)t_raw / 256.0f;

    if (!assertRange(alt,  ALT_MIN_M,  ALT_MAX_M,  ERR_SENSOR_RANGE)) { return false; }
    if (!assertRange(temp, TEMP_MIN_C, TEMP_MAX_C, ERR_SENSOR_RANGE)) { return false; }

    *alt_out  = alt;
    *temp_out = temp;
    return true;
}

bool pollBarometer(SensorData* data) {
    static float    s_baro_alt   = 0.0f;
    static float    s_baro_temp  = 25.0f;
    static bool     s_triggered  = false;
    static uint32_t s_trigger_ms = 0UL;

    if (!assertNotNull(data, ERR_SENSOR_RANGE)) { return false; }
    if (!g_mpl_ok)                              { return false; }

    if (!s_triggered) {
        triggerBarometer();
        s_triggered  = true;
        s_trigger_ms = (uint32_t)millis();
        data->altitude    = s_baro_alt;
        data->temperature = s_baro_temp;
        return true;
    }

    uint32_t now = (uint32_t)millis();
    if ((now - s_trigger_ms) < CFG_BARO_CONV_MS) {
        data->altitude    = s_baro_alt;
        data->temperature = s_baro_temp;
        return true;
    }

    uint8_t status_byte = 0U;
    uint8_t got = i2cReadBytes(I2C_ADDR_BARO, 0x00U, &status_byte, 1U);
    if (got != 1U || !(status_byte & 0x08U)) {
        if ((now - s_trigger_ms) > CFG_BARO_TIMEOUT_MS) {
            s_triggered = false;
#ifdef DEBUG
            Serial.println("Baro DRDY timeout — re-triggering");
#endif
        }
        data->altitude    = s_baro_alt;
        data->temperature = s_baro_temp;
        return true;
    }

    float new_alt = 0.0f, new_temp = 0.0f;
    if (readBaroRawBytes(&new_alt, &new_temp)) {
        s_baro_alt  = new_alt;
        s_baro_temp = new_temp;
    }

    s_triggered       = false;
    data->altitude    = s_baro_alt;
    data->temperature = s_baro_temp;
    return true;
}

// =============================================================================
//  Movement detection
// =============================================================================

bool detectMovement(float ax, float ay, float az) {
    static float s_last_ax   = 0.0f;
    static float s_last_ay   = 0.0f;
    static float s_last_az   = 0.0f;
    static bool  s_first_read = true;

    if (!assertRange(ax, ACCEL_MIN_MPS2, ACCEL_MAX_MPS2, ERR_SENSOR_RANGE)) { return false; }
    if (!assertRange(ay, ACCEL_MIN_MPS2, ACCEL_MAX_MPS2, ERR_SENSOR_RANGE)) { return false; }

    if (s_first_read) {
        s_last_ax    = ax;
        s_last_ay    = ay;
        s_last_az    = az;
        s_first_read = false;
        return true;
    }

    float delta_ax = fabsf(ax - s_last_ax);
    float delta_ay = fabsf(ay - s_last_ay);
    float delta_az = fabsf(az - s_last_az);

    s_last_ax = ax;
    s_last_ay = ay;
    s_last_az = az;

    return (delta_ax > CFG_MOVEMENT_THRESHOLD_MPS2 ||
            delta_ay > CFG_MOVEMENT_THRESHOLD_MPS2 ||
            delta_az > CFG_MOVEMENT_THRESHOLD_MPS2);
}

// =============================================================================
//  I2C bus reset (hardware bit-bang recovery)
// =============================================================================

void resetI2CBus(void) {
    if (PIN_I2C_SDA == 0U || PIN_I2C_SCL == 0U) { return; }
    if (PIN_I2C_SDA == PIN_I2C_SCL)              { return; }

    Wire.end();
    delay(50);

    pinMode(PIN_I2C_SDA, OUTPUT);
    pinMode(PIN_I2C_SCL, OUTPUT);
    digitalWrite(PIN_I2C_SDA, HIGH);
    digitalWrite(PIN_I2C_SCL, HIGH);
    delay(10);

    for (int i = 0; i < 18; i++) {
        digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(50);
        digitalWrite(PIN_I2C_SCL, LOW);  delayMicroseconds(50);
    }

    // STOP condition
    digitalWrite(PIN_I2C_SDA, LOW);  delayMicroseconds(50);
    digitalWrite(PIN_I2C_SCL, HIGH); delayMicroseconds(50);
    digitalWrite(PIN_I2C_SDA, HIGH); delayMicroseconds(50);

    delay(50);
    pinMode(PIN_I2C_SDA, INPUT_PULLUP);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);
    delay(10);

#ifdef DEBUG
    if (digitalRead(PIN_I2C_SDA) == LOW) { Serial.println("SDA stuck LOW after recovery"); }
    if (digitalRead(PIN_I2C_SCL) == LOW) { Serial.println("SCL stuck LOW after recovery"); }
#endif

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setTimeOut(100);
    delay(100);
}
