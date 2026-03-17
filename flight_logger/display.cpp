#include "display.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static Adafruit_SSD1306 g_display(DISP_WIDTH, DISP_HEIGHT, &Wire, -1);

// =============================================================================
//  Initialisation
// =============================================================================

bool initDisplay(void) {
    if (DISP_WIDTH == 0U || DISP_HEIGHT == 0U) {
        g_error_state = ERR_DISPLAY_INIT;
        return false;
    }
    for (int retry = 0; retry < CFG_MAX_RETRIES; retry++) {
        if (g_display.begin(SSD1306_SWITCHCAPVCC, DISP_ADDR)) {
            g_display.clearDisplay();
            g_display.setTextSize(1);
            g_display.setTextColor(SSD1306_WHITE);
            g_display.setCursor(0, 0);
            (void)g_display.println("Display Ready");
            g_display.display();
            return true;
        }
        delay(100);
    }
    g_error_state = ERR_DISPLAY_INIT;
    return false;
}

// =============================================================================
//  Per-loop update
// =============================================================================

void updateDisplay(const SensorData* data, bool moving, unsigned long idle_ms) {
    if (!g_display_ok)                         { return; }
    if (!assertNotNull(data, ERR_SENSOR_RANGE)) { return; }

    g_display.clearDisplay();
    g_display.setCursor(0, 0);

    g_display.print("A:");
    g_display.print(data->ax, 0);
    g_display.print(" ");
    g_display.print(data->ay, 0);
    g_display.print(" ");
    (void)g_display.println(data->az, 0);

    g_display.print("G:");
    g_display.print(data->gx, 0);
    g_display.print(" ");
    g_display.print(data->gy, 0);
    g_display.print(" ");
    (void)g_display.println(data->gz, 0);

    g_display.print("Alt:");
    g_display.print(data->altitude, 1);
    g_display.print("m T:");
    (void)g_display.println(data->temperature, 1);

    g_display.print("Time:");
    (void)g_display.println(data->timestamp / 1000UL);

    g_display.print("MOV:");
    (void)g_display.println(moving ? "YES" : "NO ");

    long remaining = ((long)CFG_MIN_AWAKE_MS - (long)idle_ms) / 1000L;
    if (remaining < 0L) { remaining = 0L; }
    g_display.print("Slp in:");
    (void)g_display.println(remaining);

    g_display.display();
}

// =============================================================================
//  Status screens
// =============================================================================

void displaySensorFail(bool imu_ok, bool baro_ok) {
    if (!g_display_ok) { return; }
    g_display.clearDisplay();
    g_display.setCursor(0, 0);
    g_display.print("Sensor fail:");
    if (!imu_ok)  { g_display.print(" IMU");  }
    if (!baro_ok) { g_display.print(" BARO"); }
    g_display.display();
}

void displayEnterSleep(void) {
    if (!g_display_ok) { return; }
    g_display.clearDisplay();
    g_display.setCursor(0, 0);
    (void)g_display.println("Entering Sleep");
    g_display.display();
}
