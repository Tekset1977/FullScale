#include "sleep.h"
#include "sensors.h"
#include "storage.h"
#include "display.h"
#include <Wire.h>
#include <esp_sleep.h>

// =============================================================================
//  Wake helpers (private)
// =============================================================================

static void wakeReinitPeripherals(void) {
    if (PIN_I2C_SDA == 0U || PIN_I2C_SCL == 0U) { return; }
    if (CFG_SLEEP_DURATION_SEC == 0UL)           { return; }

    resetI2CBus();
#ifdef DEBUG
    Serial.println("I2C bus reset done");
#endif

    bool icm_reset = initIMU();
#ifdef DEBUG
    Serial.printf("IMU: %s\n", icm_reset ? "OK" : "FAIL");
#endif
    g_icm_ok = icm_reset;

    g_mpl_ok = initBarometer();
#ifdef DEBUG
    Serial.printf("BARO: %s\n", g_mpl_ok ? "OK" : "FAIL");
#endif
    if (g_mpl_ok) { delay(150); }

    g_display_ok = initDisplay();
#ifdef DEBUG
    Serial.printf("DISP: %s\n", g_display_ok ? "OK" : "FAIL");
#endif
}

// =============================================================================
//  Public API
// =============================================================================

void enterLightSleep(void) {
    if (CFG_SLEEP_DURATION_SEC == 0UL) { return; }
    if (CFG_MIN_AWAKE_MS == 0UL)       { return; }

    displayEnterSleep();
    storageCloseFiles();

    esp_sleep_enable_timer_wakeup(
        (uint64_t)CFG_SLEEP_DURATION_SEC * 1000000ULL);
#ifdef DEBUG
    Serial.println("sleeping...");
    Serial.flush();
#endif

    esp_light_sleep_start();

#ifdef DEBUG
    Serial.println("woke up");
#endif

    g_wake_time_ms = (unsigned long)millis();

    wakeReinitPeripherals();
    storageReopenFiles();
}

void updateAltitudeIdleTimer(float altitude_m, unsigned long* wake_time_ms) {
    static float s_alt_ref_m    = 0.0f;
    static bool  s_alt_ref_init = false;

    if (!assertNotNull(wake_time_ms, ERR_NONE))                              { return; }
    if (!assertRange(altitude_m, ALT_MIN_M, ALT_MAX_M, ERR_SENSOR_RANGE))   { return; }

    if (!s_alt_ref_init) {
        s_alt_ref_m    = altitude_m;
        s_alt_ref_init = true;
        return;
    }

    float delta = fabsf(altitude_m - s_alt_ref_m);
    if (delta >= CFG_ALT_WAKE_THRESHOLD_M) {
        *wake_time_ms = (unsigned long)millis();
        s_alt_ref_m   = altitude_m;
#ifdef DEBUG
        Serial.printf("Alt change %.1fm — resetting idle timer\n", delta);
#endif
    }
}
