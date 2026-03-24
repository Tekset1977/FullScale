#pragma once
#include <Arduino.h>

// Comment out to disable all debug serial output
//#define DEBUG

// -----------------------------------------------------------------------------
//  Hardware pin constants
// -----------------------------------------------------------------------------
static const uint8_t  PIN_SD_CS    = 5U;
static const uint8_t  PIN_SPI_SCK  = 18U;
static const uint8_t  PIN_SPI_MISO = 19U;
static const uint8_t  PIN_SPI_MOSI = 23U;
static const uint8_t  PIN_I2C_SDA  = 21U;
static const uint8_t  PIN_I2C_SCL  = 22U;
static const uint8_t  PIN_SERVO    = 4U;

// -----------------------------------------------------------------------------
//  Timing / retry constants
// -----------------------------------------------------------------------------
static const uint8_t  CFG_MAX_RETRIES        = 5U;
static const uint32_t CFG_FLUSH_INTERVAL_MS  = 2000UL;
static const uint32_t CFG_LOOP_DELAY_MS      = 20UL;
static const uint32_t CFG_BARO_CONV_MS       = 8UL;
static const uint32_t CFG_SLEEP_DURATION_SEC = 5UL;
static const uint32_t CFG_MIN_AWAKE_MS       = 5000UL;
static const uint32_t CFG_BARO_TIMEOUT_MS    = 200UL;

// -----------------------------------------------------------------------------
//  Movement / altitude thresholds
// -----------------------------------------------------------------------------
static const float    CFG_MOVEMENT_THRESHOLD_MPS2   = 200.0f;
static const float    CFG_ALT_WAKE_THRESHOLD_M      = 10.0f;
static const uint32_t CFG_POST_LANDING_AWAKE_MS     = 15UL * 60UL * 1000UL;
static const float    CFG_LANDING_ALT_THRESHOLD_M   = 5.0f;

// -----------------------------------------------------------------------------
//  Display
// -----------------------------------------------------------------------------
static const uint8_t  DISP_WIDTH   = 128U;
static const uint8_t  DISP_HEIGHT  = 64U;
static const uint8_t  DISP_ADDR    = 0x3CU;

// -----------------------------------------------------------------------------
//  Sensor validity ranges
// -----------------------------------------------------------------------------
static const float    ACCEL_MIN_MPS2 = -2000.0f;
static const float    ACCEL_MAX_MPS2 =  2000.0f;
static const float    GYRO_MIN_DPS   = -2000.0f;
static const float    GYRO_MAX_DPS   =  2000.0f;
static const float    ALT_MIN_M      =  -500.0f;
static const float    ALT_MAX_M      = 10000.0f;
static const float    TEMP_MIN_C     =   -50.0f;
static const float    TEMP_MAX_C     =   100.0f;

// -----------------------------------------------------------------------------
//  Servo geometry
// -----------------------------------------------------------------------------
static const uint32_t PWM_FREQ_HZ  = 333UL;
static const uint8_t  PWM_RES_BITS = 16U;
static const uint32_t SERVO_MOVE_MS = 500UL;
static const uint32_t PULSE_MIN_US  = 800UL;
static const uint32_t PULSE_MAX_US  = 2200UL;
static const int      SERVO_ANGLE_MIN = -60;
static const int      SERVO_ANGLE_MAX =  60;

// ============= LINES FOR EDIT ============
static const int      SERVO_ANGLE_INIT =  50;   // home position
static const int      SERVO_ANGLE_GO   =  10;   // release position

static const float    ALT_BAND_LO_M = 200.0f * 0.3048f;   // feet → metres
static const float    ALT_BAND_HI_M = 800.0f * 0.3048f;
// =========================================

// -----------------------------------------------------------------------------
//  Error codes
// -----------------------------------------------------------------------------
static const int ERR_NONE         = 0;
static const int ERR_SD_INIT      = 1;
static const int ERR_IMU_INIT     = 2;
static const int ERR_BARO_INIT    = 3;
static const int ERR_FILE_OPEN    = 4;
static const int ERR_SENSOR_RANGE = 5;
static const int ERR_WRITE_FAILED = 6;
static const int ERR_DISPLAY_INIT = 7;
static const int ERR_NULL_PTR     = 8;

// -----------------------------------------------------------------------------
//  I2C addresses
// -----------------------------------------------------------------------------
static const uint8_t I2C_ADDR_ICM  = 0x69U;
static const uint8_t I2C_ADDR_BARO = 0x60U;

// -----------------------------------------------------------------------------
//  Ring buffer sizes
//  ICM_RING_SIZE × 20 ms loop = 640 ms headroom before overrun
//  MPL_RING_SIZE × 20 ms loop = 320 ms headroom (baro is slower / cached)
// -----------------------------------------------------------------------------
static const uint8_t ICM_RING_SIZE = 32U;
static const uint8_t MPL_RING_SIZE = 16U;


//------------------------------
//DC Motors stuff 
//-------------
static const uint8_t  PIN_MOT_A_IN1 = 25U;
static const uint8_t  PIN_MOT_A_IN2 = 26U;
static const uint8_t  PIN_MOT_B_IN1 = 32U;
static const uint8_t  PIN_MOT_B_IN2 = 33U;
static const uint8_t  PWM_MOT_CHAN_A = 2U;
static const uint8_t  PWM_MOT_CHAN_B = 3U;
static const uint8_t  PWM_MOT_RES    = 8U;   // 0–255
static const uint32_t PWM_MOT_FREQ   = 20000UL;

// -----------------------------------------------------------------------------
//  Data structures
// -----------------------------------------------------------------------------
struct AccelCal {
    float offset_x, offset_y, offset_z;
    float scale_x,  scale_y,  scale_z;
};

struct SensorData {
    unsigned long timestamp;
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float altitude;
    float temperature;
};

struct IcmSample {
    unsigned long timestamp;
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
};

struct MplSample {
    unsigned long timestamp;
    float altitude;
    float temperature;
};

struct IcmRingBuf {
    IcmSample data[ICM_RING_SIZE];
    uint8_t   head;   // next write index
    uint8_t   tail;   // next read  index
    uint8_t   count;  // entries currently held
};

struct MplRingBuf {
    MplSample data[MPL_RING_SIZE];
    uint8_t   head;
    uint8_t   tail;
    uint8_t   count;
};
