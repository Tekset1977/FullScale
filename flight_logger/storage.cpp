#include "storage.h"
#include "buffers.h"
#include <SPI.h>
#include <SD.h>

// -----------------------------------------------------------------------------
//  Module-private hardware objects
// -----------------------------------------------------------------------------
static SPIClass g_spi(VSPI);
static File     g_logFile;
static File     g_servoFile;
static File     g_soilFile;   // FIX 4: added for soil log

// File-valid assertion — private to this module
static bool assertFileValid(void) {
    if (!g_logFile) {
        g_error_state = ERR_WRITE_FAILED;
        return false;
    }
    return true;
}

// =============================================================================
//  SPI / SD initialisation
// =============================================================================

void storageBeginSPI(void) {
    g_spi.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
}

bool initSD(void) {
    if (PIN_SD_CS == 0U) {
        g_error_state = ERR_SD_INIT;
        return false;
    }
    for (int retry = 0; retry < CFG_MAX_RETRIES; retry++) {
        if (SD.begin(PIN_SD_CS, g_spi)) { return true; }
        delay(100);
    }
    g_error_state = ERR_SD_INIT;
    return false;
}

// =============================================================================
//  Log file initialisation
// =============================================================================

bool initLogFile(void) {
    static const char* const HEADER =
        "timestamp_ms,ax,ay,az,gx,gy,gz,mx,my,mz,alt_m,temp_C";

    if (HEADER == NULL || HEADER[0] == '\0') {
        g_error_state = ERR_FILE_OPEN;
        return false;
    }
    for (int retry = 0; retry < CFG_MAX_RETRIES; retry++) {
        g_logFile = SD.open("/imu_log.csv", FILE_WRITE);
        if (!g_logFile) { delay(100); continue; }

        size_t written = g_logFile.println(HEADER);
        if (written == 0U) { g_logFile.close(); delay(100); continue; }

        g_logFile.flush();

        g_servoFile = SD.open("/servo_log.csv", FILE_WRITE);
        if (!g_servoFile) { g_logFile.close(); delay(100); continue; }

        written = g_servoFile.println("timestamp_ms,altitude_m");
        if (written == 0U) { g_servoFile.close(); g_logFile.close(); delay(100); continue; }

        g_servoFile.flush();
        return true;
    }
    g_error_state = ERR_FILE_OPEN;
    return false;
}

// FIX 4: initSoilLogFile — was declared in storage.h but never implemented
bool initSoilLogFile(void) {
    static const char* const HEADER =
        "timestamp_ms,moisture,stemma_temp_c,ec_uScm,ph,n_mgkg,modbus,dataq";

    for (int retry = 0; retry < CFG_MAX_RETRIES; retry++) {
        g_soilFile = SD.open("/soil_log.csv", FILE_WRITE);
        if (!g_soilFile) { delay(100); continue; }

        size_t written = g_soilFile.println(HEADER);
        if (written == 0U) { g_soilFile.close(); delay(100); continue; }

        g_soilFile.flush();
        return true;
    }
    g_error_state = ERR_FILE_OPEN;
    return false;
}

// =============================================================================
//  Per-loop writes
// =============================================================================

bool writeLogData(const SensorData* data) {
    if (!assertNotNull(data, ERR_WRITE_FAILED)) { return false; }
    if (!assertFileValid())                      { return false; }

    char line[160];
    int len = snprintf(line, sizeof(line),
        "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f",
        data->timestamp,
        data->ax, data->ay, data->az,
        data->gx, data->gy, data->gz,
        data->mx, data->my, data->mz,
        data->altitude, data->temperature);

    if (len < 0 || len >= (int)sizeof(line)) {
        g_error_state = ERR_WRITE_FAILED;
        return false;
    }
    size_t written = g_logFile.println(line);
    if (written == 0U) {
        g_error_state = ERR_WRITE_FAILED;
        return false;
    }
    return true;
}

bool writeServoEvent(float altitude_m) {
    if (!g_servoFile) {
        g_error_state = ERR_WRITE_FAILED;
        return false;
    }
    if (!assertRange(altitude_m, ALT_MIN_M, ALT_MAX_M, ERR_SENSOR_RANGE)) { return false; }

    char line[64];
    int len = snprintf(line, sizeof(line), "%lu,%.3f",
                       (unsigned long)millis(), altitude_m);

    if (len < 0 || len >= (int)sizeof(line)) {
        g_error_state = ERR_WRITE_FAILED;
        return false;
    }
    size_t written = g_servoFile.println(line);
    if (written == 0U) {
        g_error_state = ERR_WRITE_FAILED;
        return false;
    }
    g_servoFile.flush();   // flush immediately — event must survive a crash
    return true;
}

// FIX 4: writeSoilData — was declared in storage.h but never implemented
bool writeSoilData(const SoilSample* s) {
    if (!assertNotNull(s, ERR_WRITE_FAILED)) { return false; }
    if (!g_soilFile) {
        g_error_state = ERR_WRITE_FAILED;
        return false;
    }

    char line[128];
    int len = snprintf(line, sizeof(line),
        "%lu,%.3f,%.1f,%.0f,%.1f,%.0f,%s,%s",
        s->timestamp,
        s->moisture,
        s->stemma_temp_c,
        s->ec_uScm,
        s->ph,
        s->n_mgkg,
        s->modbus_tag,
        s->dataq);

    if (len < 0 || len >= (int)sizeof(line)) {
        g_error_state = ERR_WRITE_FAILED;
        return false;
    }
    size_t written = g_soilFile.println(line);
    if (written == 0U) {
        g_error_state = ERR_WRITE_FAILED;
        return false;
    }
    return true;
}

// Drain both ring buffers and write paired rows to the SD log.
// Stops when either buffer runs dry so the two streams stay in lock-step.
void drainBufsToLog(void) {
    if (!assertFileValid()) { return; }

    while (g_icm_buf.count > 0U && g_mpl_buf.count > 0U) {
        IcmSample icm;
        MplSample mpl;
        if (!icmBufPop(&icm)) { break; }
        if (!mplBufPop(&mpl)) { break; }

        SensorData d;
        d.timestamp   = icm.timestamp;
        d.ax = icm.ax;  d.ay = icm.ay;  d.az = icm.az;
        d.gx = icm.gx;  d.gy = icm.gy;  d.gz = icm.gz;
        d.mx = icm.mx;  d.my = icm.my;  d.mz = icm.mz;
        d.altitude    = mpl.altitude;
        d.temperature = mpl.temperature;

        bool ok = writeLogData(&d);
#ifdef DEBUG
        if (!ok) { Serial.println("drainBufsToLog: writeLogData failed"); }
#endif
    }
}

void flushLogFile(void) {
    static uint32_t s_last_flush_ms = 0UL;
    if (!g_logFile)                   { return; }
    if (CFG_FLUSH_INTERVAL_MS == 0UL) { return; }

    uint32_t now = (uint32_t)millis();
    if ((now >= s_last_flush_ms) &&
        ((now - s_last_flush_ms) > CFG_FLUSH_INTERVAL_MS)) {
        g_logFile.flush();
        s_last_flush_ms = now;
    }
}

// =============================================================================
//  Sleep / wake helpers
// =============================================================================

void storageCloseFiles(void) {
    if (g_logFile)   { g_logFile.flush();   g_logFile.close();   }
    if (g_servoFile) { g_servoFile.flush(); g_servoFile.close(); }
    if (g_soilFile)  { g_soilFile.flush();  g_soilFile.close();  }  // FIX 5: was missing
}

void storageReopenFiles(void) {
    if (PIN_SD_CS == 0U) { return; }
    if (g_logFile)   { g_logFile.close();   }
    if (g_servoFile) { g_servoFile.close(); }
    if (g_soilFile)  { g_soilFile.close();  }  // FIX 5: was missing

    if (!SD.begin(PIN_SD_CS, g_spi)) {
#ifdef DEBUG
        Serial.println("SD re-init FAIL");
#endif
        return;
    }
#ifdef DEBUG
    Serial.println("SD re-init OK");
#endif

    g_logFile = SD.open("/imu_log.csv", FILE_APPEND);
#ifdef DEBUG
    if (!g_logFile) { Serial.println("Log file reopen FAIL"); }
    else            { Serial.println("Log file reopen OK");   }
#endif

    g_servoFile = SD.open("/servo_log.csv", FILE_APPEND);
#ifdef DEBUG
    if (!g_servoFile) { Serial.println("Servo file reopen FAIL"); }
    else              { Serial.println("Servo file reopen OK");   }
#endif

    // FIX 5: soil file reopen was missing
    // Only reopen if it was previously opened (i.e. ground stage has started)
    if (g_soilFile) {
        g_soilFile = SD.open("/soil_log.csv", FILE_APPEND);
#ifdef DEBUG
        if (!g_soilFile) { Serial.println("Soil file reopen FAIL"); }
        else             { Serial.println("Soil file reopen OK");   }
#endif
    }
}
