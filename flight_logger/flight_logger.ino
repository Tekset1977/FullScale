// -----------------------------------------------------------------------------
//  External library includes
// -----------------------------------------------------------------------------
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// -----------------------------------------------------------------------------
//  Project module includes
// -----------------------------------------------------------------------------
#include "config.h"
#include "globals.h"
#include "buffers.h"
#include "sensors.h"
#include "storage.h"
#include "display.h"
#include "servo.h"
#include "sleep.h"

// =============================================================================
//  setup()
// =============================================================================
void setup(void) {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    Wire.setTimeOut(100);
    delay(500);
    Serial.begin(115200);
    delay(500);

    // Servo — attach LEDC channel before anything moves
    bool ledc_ok = ledcAttach(PIN_SERVO, PWM_FREQ_HZ, PWM_RES_BITS);
    if (!ledc_ok) {
#ifdef DEBUG
        Serial.println("LEDC attach FAIL");
#endif
    } else {
        bool servo_ok = writeServoAngle(SERVO_ANGLE_INIT);
#ifdef DEBUG
        if (!servo_ok) { Serial.println("Servo init write FAIL"); }
#endif
    }

    // Sanity checks before touching any bus
    if (PIN_SPI_SCK == PIN_SPI_MISO || PIN_SPI_SCK == PIN_SPI_MOSI) {
#ifdef DEBUG
        Serial.println("SPI pin config error — halting");
#endif
        return;
    }
    if (Wire.getTimeOut() == 0U) {
#ifdef DEBUG
        Serial.println("Wire timeout not set — halting");
#endif
        return;
    }

#ifdef DEBUG
    Serial.println("Scanning I2C...");
    for (uint8_t addr = 1U; addr < 127U; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0U) {
            Serial.printf("  0x%02X\n", addr);
        }
    }
    Serial.println("Scan done.");
#endif

    storageBeginSPI();

    g_display_ok = initDisplay();
    g_display_ok = true;   // keep true even if OLED absent

    if (!initSD())        { return; }

    if (!initIMU())       { return; }
    g_icm_ok = true;

    if (!initBarometer()) { return; }
    g_mpl_ok = true;

    g_alt_baseline_m = baroReadAltitude();
#ifdef DEBUG
    Serial.printf("Baseline altitude: %.2f m\n", g_alt_baseline_m);
#endif

    if (!initLogFile())   { return; }

    delay(100);
    // enterLightSleep();   // uncomment to sleep on first boot
}

// =============================================================================
//  loop()
// =============================================================================
void loop(void) {
#ifdef DEBUG
    Serial.printf("tick — icm:%d mpl:%d\n", g_icm_ok, g_mpl_ok);
#endif

    // Collect a fresh reading from each sensor
    SensorData data;
    data.timestamp   = (unsigned long)millis();
    data.ax = data.ay = data.az = 0.0f;
    data.gx = data.gy = data.gz = 0.0f;
    data.mx = data.my = data.mz = 0.0f;
    data.altitude    = 0.0f;
    data.temperature = 25.0f;

    bool imu_ok  = readIMUSensors(&data);
    bool baro_ok = pollBarometer(&data);

    data.altitude -= g_alt_baseline_m;

    // Clamp error state to known range
    if (g_error_state < ERR_NONE || g_error_state > ERR_DISPLAY_INIT) {
        g_error_state = ERR_NONE;
    }

    if (!imu_ok || !baro_ok) {
        displaySensorFail(imu_ok, baro_ok);
#ifdef DEBUG
        Serial.printf("Fail — IMU:%d BARO:%d err:%d\n",
                      imu_ok, baro_ok, g_error_state);
#endif
        delay(CFG_LOOP_DELAY_MS);
        return;
    }

    updateAltitudeServo(data.altitude);

    bool moving = detectMovement(data.ax, data.ay, data.az);
    if (moving) { g_wake_time_ms = (unsigned long)millis(); }

    updateAltitudeIdleTimer(data.altitude, &g_wake_time_ms);

    unsigned long awake_ms = (unsigned long)millis() - g_wake_time_ms;

    // Push sensor reads into their respective ring buffers
    IcmSample icm_s = {
        data.timestamp,
        data.ax, data.ay, data.az,
        data.gx, data.gy, data.gz,
        data.mx, data.my, data.mz
    };
    icmBufPush(&icm_s);

    MplSample mpl_s = {
        data.timestamp,
        data.altitude,
        data.temperature
    };
    mplBufPush(&mpl_s);

    // Drain both ring buffers to the SD card
    drainBufsToLog();

    updateDisplay(&data, moving, awake_ms);
    flushLogFile();

    // if (awake_ms > CFG_MIN_AWAKE_MS) { enterLightSleep(); }
    delay(CFG_LOOP_DELAY_MS);
}
