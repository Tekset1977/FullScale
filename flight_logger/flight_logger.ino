#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <Adafruit_MPL3115A2.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --------------------------
//  Project module includes
// ------------------------------------------
#include "config.h"
#include "globals.h"
#include "buffers.h"
#include "sensors.h"
#include "storage.h"
#include "display.h"
#include "servo.h"
#include "sleep.h"
#include "stage.h"   // FlightStage enum + g_stage
#include "soil.h"

// Forward declarations
static void flightLoop(SensorData* data);
static void groundLoop(SensorData* data);
static void initGroundStage(void);


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

    g_stage = STAGE_FLIGHT;

    delay(100);
    // enterLightSleep();   // uncomment to sleep on first boot
}

// loop() — shared sensor reads, then stage dispatch
void loop(void) {
#ifdef DEBUG
    Serial.printf("tick — icm:%d mpl:%d  stage:%d\n",
                  g_icm_ok, g_mpl_ok, (int)g_stage);
#endif

    // Shared — runs every tick regardless of stage
    SensorData data;
    data.timestamp   = (unsigned long)millis();
    data.ax = data.ay = data.az = 0.0f;
    data.gx = data.gy = data.gz = 0.0f;
    data.mx = data.my = data.mz = 0.0f;
    data.altitude    = 0.0f;
    data.temperature = 25.0f;

    bool imu_ok  = readIMUSensors(&data);
    bool baro_ok = pollBarometer(&data);

    // Subtract baseline once here so both stage functions see relative altitude
    data.altitude -= g_alt_baseline_m;

    // FIX 6: clamp was ERR_DISPLAY_INIT (7) — ERR_NULL_PTR/ERR_SOIL_* (8-10) were
    // silently reset to ERR_NONE. Now uses ERR_MAX which is kept in sync in config.h.
    if (g_error_state < ERR_NONE || g_error_state > ERR_MAX) {
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

    // Stage dispatch
    switch (g_stage) {
        case STAGE_FLIGHT: flightLoop(&data); break;
        case STAGE_GROUND: groundLoop(&data); break;
        default:                              break;
    }

    delay(CFG_LOOP_DELAY_MS);
}

// flightLoop() — original behaviour
static void flightLoop(SensorData* data) {

    // Landing detection and stage transition
    if (!g_landing_confirmed &&
        servoHasActuated() &&
        data->altitude < CFG_LANDING_ALT_THRESHOLD_M) {
        g_landing_confirmed = true;
        g_landing_time_ms   = millis();
#ifdef DEBUG
        Serial.printf("Landing detected — alt=%.2f m  ts=%lu ms\n",
                      data->altitude, g_landing_time_ms);
#endif
    }

    if (g_landing_confirmed) {
        unsigned long since_landing = millis() - g_landing_time_ms;
        if (since_landing >= CFG_POST_LANDING_AWAKE_MS) {
            g_stage = STAGE_GROUND;
            initGroundStage();
            // Return immediately so flight logic does not run on the
            // transition tick. groundLoop picks up on the next loop() call.
            return;
        }
    }

    // Flight operations
    updateAltitudeServo(data->altitude);

    bool moving = detectMovement(data->ax, data->ay, data->az);
    if (moving) { g_wake_time_ms = (unsigned long)millis(); }

    updateAltitudeIdleTimer(data->altitude, &g_wake_time_ms);

    unsigned long awake_ms = (unsigned long)millis() - g_wake_time_ms;

    IcmSample icm_s = {
        data->timestamp,
        data->ax, data->ay, data->az,
        data->gx, data->gy, data->gz,
        data->mx, data->my, data->mz
    };
    icmBufPush(&icm_s);

    MplSample mpl_s = {
        data->timestamp,
        data->altitude,
        data->temperature
    };
    mplBufPush(&mpl_s);

    drainBufsToLog();
    updateDisplay(data, moving, awake_ms);
    flushLogFile();

    // if (awake_ms > CFG_MIN_AWAKE_MS) { enterLightSleep(); }
}

// groundLoop() — IMU/baro logging + 1 Hz soil reads
static void groundLoop(SensorData* data) {
    // TODO: 2-servo stabilisation
    // TODO: 2x DC motor drive

    // IMU/baro logging carries over so the SD record is uninterrupted
    IcmSample icm_s = {
        data->timestamp,
        data->ax, data->ay, data->az,
        data->gx, data->gy, data->gz,
        data->mx, data->my, data->mz
    };
    icmBufPush(&icm_s);

    MplSample mpl_s = {
        data->timestamp,
        data->altitude,
        data->temperature
    };
    mplBufPush(&mpl_s);

    drainBufsToLog();
    flushLogFile();

    // Soil — read at 1 Hz, not every 20 ms tick
    static uint32_t s_last_soil_ms = 0UL;
    uint32_t now = (uint32_t)millis();
    if ((now - s_last_soil_ms) >= SOIL_LOG_INTERVAL_MS) {
        s_last_soil_ms = now;
        SoilSample soil;
        if (readSoilSensors(&soil)) {
            writeSoilData(&soil);
        }
    }
}

// initGroundStage() — called once on STAGE_FLIGHT -> STAGE_GROUND transition
static void initGroundStage(void) {
#ifdef DEBUG
    Serial.println(">>> Entering STAGE_GROUND");
#endif

    if (!initSoilSensors()) {
        // Log the failure but don't halt — IMU/baro logging should continue
#ifdef DEBUG
        Serial.printf("Soil init failed, err=%d\n", g_error_state);
#endif
    }
    initSoilLogFile();
    // TODO: attach stabiliser servo LEDC channels
    // TODO: initMotors()
    // TODO: resetStabilizer()
}
