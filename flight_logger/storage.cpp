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
static const char* const IMU_LOG_HEADER =
    "timestamp_ms,ax,ay,az,gx,gy,gz,alt_m,temp_C";
static const char* const SERVO_LOG_HEADER = "timestamp_ms,altitude_m";
static const char* const SOIL_LOG_HEADER =
    "timestamp_ms,moisture,stemma_temp_c,ec_uScm,ph,n_mgkg,modbus,dataq";
static const char* const IMU_LOG_PATH = "/imu_log.csv";
static const char* const SERVO_LOG_PATH = "/servo_log.csv";
static const char* const SOIL_LOG_PATH = "/soil_log.csv";
static const char* const IMU_RECOVERY_FMT = "/imu_log_recover_%03u.csv";
static char     s_active_log_path[32] = "/imu_log.csv";
static uint8_t  s_consecutive_log_failures = 0U;
static uint16_t s_recovery_log_index = 0U;

// File-valid assertion — private to this module
static bool assertFileValid(void) {
    if (!g_logFile) {
        g_error_state = ERR_WRITE_FAILED;
        return false;
    }
    return true;
}

static bool setActiveLogPath(const char* path) {
    if (!assertNotNull(path, ERR_NULL_PTR)) { return false; }

    int len = snprintf(s_active_log_path, sizeof(s_active_log_path), "%s", path);
    if (len < 0 || len >= (int)sizeof(s_active_log_path)) {
        g_error_state = ERR_FILE_OPEN;
        return false;
    }
    return true;
}

static bool openCsvWithHeader(const char* path, const char* header, File* out_file) {
    if (!assertNotNull(path, ERR_NULL_PTR))    { return false; }
    if (!assertNotNull(header, ERR_NULL_PTR))  { return false; }
    if (!assertNotNull(out_file, ERR_NULL_PTR)) { return false; }

    *out_file = SD.open(path, FILE_WRITE);
    if (!*out_file) { return false; }

    size_t written = out_file->println(header);
    if (written == 0U) {
        out_file->close();
        g_error_state = ERR_FILE_OPEN;
        return false;
    }

    out_file->flush();
    return true;
}

static bool openAppendFile(const char* path, File* out_file) {
    if (!assertNotNull(path, ERR_NULL_PTR))    { return false; }
    if (!assertNotNull(out_file, ERR_NULL_PTR)) { return false; }

    *out_file = SD.open(path, FILE_APPEND);
    return (bool)(*out_file);
}

static bool nextRecoveryLogPath(char* path_out, size_t path_out_size) {
    if (!assertNotNull(path_out, ERR_NULL_PTR)) { return false; }
    if (path_out_size == 0U) {
        g_error_state = ERR_FILE_OPEN;
        return false;
    }

    for (uint16_t idx = (uint16_t)(s_recovery_log_index + 1U); idx < 1000U; idx++) {
        int len = snprintf(path_out, path_out_size, IMU_RECOVERY_FMT, idx);
        if (len < 0 || len >= (int)path_out_size) {
            g_error_state = ERR_FILE_OPEN;
            return false;
        }
        if (!SD.exists(path_out)) {
            s_recovery_log_index = idx;
            return true;
        }
    }

    g_error_state = ERR_FILE_OPEN;
    return false;
}

static bool recoverPrimaryLogFile(void) {
    char recovery_path[32];
    bool had_soil_file = (bool)g_soilFile;

    if (g_logFile)   { g_logFile.flush();   g_logFile.close();   }
    if (g_servoFile) { g_servoFile.flush(); g_servoFile.close(); }
    if (g_soilFile)  { g_soilFile.flush();  g_soilFile.close();  }

    if (!initSD()) { return false; }
    if (!nextRecoveryLogPath(recovery_path, sizeof(recovery_path))) { return false; }
    if (!openCsvWithHeader(recovery_path, IMU_LOG_HEADER, &g_logFile)) { return false; }
    if (!setActiveLogPath(recovery_path)) {
        g_logFile.close();
        return false;
    }

    g_servoFile = File();
    (void)openAppendFile(SERVO_LOG_PATH, &g_servoFile);

    if (had_soil_file) {
        g_soilFile = File();
        (void)openAppendFile(SOIL_LOG_PATH, &g_soilFile);
    }

    s_consecutive_log_failures = 0U;

#ifdef DEBUG
    Serial.printf("Logging recovered to %s\n", s_active_log_path);
    if (!g_servoFile) { Serial.println("Servo file reopen FAIL after recovery"); }
    if (had_soil_file && !g_soilFile) { Serial.println("Soil file reopen FAIL after recovery"); }
#endif
    return true;
}

static bool handleConsecutiveLogFailure(void) {
    if (s_consecutive_log_failures < 255U) {
        s_consecutive_log_failures++;
    }

#ifdef DEBUG
    Serial.printf("Log write failure #%u\n", s_consecutive_log_failures);
#endif

    if (s_consecutive_log_failures < CFG_MAX_RETRIES) { return false; }

#ifdef DEBUG
    Serial.println("Attempting SD log recovery");
#endif
    return recoverPrimaryLogFile();
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
    if (IMU_LOG_HEADER == NULL || IMU_LOG_HEADER[0] == '\0') {
        g_error_state = ERR_FILE_OPEN;
        return false;
    }

    s_consecutive_log_failures = 0U;
    s_recovery_log_index = 0U;
    if (!setActiveLogPath(IMU_LOG_PATH)) { return false; }

    for (int retry = 0; retry < CFG_MAX_RETRIES; retry++) {
        if (!openCsvWithHeader(IMU_LOG_PATH, IMU_LOG_HEADER, &g_logFile)) {
            delay(100);
            continue;
        }

        if (!openCsvWithHeader(SERVO_LOG_PATH, SERVO_LOG_HEADER, &g_servoFile)) {
            g_logFile.close();
            delay(100);
            continue;
        }

        return true;
    }
    g_error_state = ERR_FILE_OPEN;
    return false;
}

// FIX 4: initSoilLogFile — was declared in storage.h but never implemented
bool initSoilLogFile(void) {
    for (int retry = 0; retry < CFG_MAX_RETRIES; retry++) {
        if (!openCsvWithHeader(SOIL_LOG_PATH, SOIL_LOG_HEADER, &g_soilFile)) {
            delay(100);
            continue;
        }

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
        "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f",
        data->timestamp,
        data->ax, data->ay, data->az,
        data->gx, data->gy, data->gz,
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
    while (g_icm_buf.count > 0U && g_mpl_buf.count > 0U) {
        if (!assertFileValid()) {
            (void)handleConsecutiveLogFailure();
            break;
        }

        IcmSample icm;
        MplSample mpl;
        if (!icmBufPeek(&icm)) { break; }
        if (!mplBufPeek(&mpl)) { break; }

        SensorData d;
        d.timestamp   = icm.timestamp;
        d.ax = icm.ax;  d.ay = icm.ay;  d.az = icm.az;
        d.gx = icm.gx;  d.gy = icm.gy;  d.gz = icm.gz;
        d.altitude    = mpl.altitude;
        d.temperature = mpl.temperature;

        if (!writeLogData(&d)) {
#ifdef DEBUG
            Serial.println("drainBufsToLog: writeLogData failed");
#endif
            (void)handleConsecutiveLogFailure();
            break;
        }

        s_consecutive_log_failures = 0U;
        if (!icmBufCommitPeek()) {
            g_error_state = ERR_WRITE_FAILED;
            break;
        }
        if (!mplBufCommitPeek()) {
            g_error_state = ERR_WRITE_FAILED;
            break;
        }
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
    bool had_soil_file = (bool)g_soilFile;

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

    g_logFile = SD.open(s_active_log_path, FILE_APPEND);
#ifdef DEBUG
    if (!g_logFile) { Serial.println("Log file reopen FAIL"); }
    else            { Serial.println("Log file reopen OK");   }
#endif

    g_servoFile = SD.open(SERVO_LOG_PATH, FILE_APPEND);
#ifdef DEBUG
    if (!g_servoFile) { Serial.println("Servo file reopen FAIL"); }
    else              { Serial.println("Servo file reopen OK");   }
#endif

    // FIX 5: soil file reopen was missing
    // Only reopen if it was previously opened (i.e. ground stage has started)
    if (had_soil_file) {
        g_soilFile = SD.open(SOIL_LOG_PATH, FILE_APPEND);
#ifdef DEBUG
        if (!g_soilFile) { Serial.println("Soil file reopen FAIL"); }
        else             { Serial.println("Soil file reopen OK");   }
#endif
    }
}
