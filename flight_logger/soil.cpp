#include "soil.h"
#include <Wire.h>
#include "Adafruit_seesaw.h"
#include <ModbusMaster.h>

// -----------------------------------------------------------------------------
//  Module-private hardware objects
// -----------------------------------------------------------------------------
static Adafruit_seesaw g_stemma;
static ModbusMaster    g_modbus;
static bool            s_stemma_ok  = false;
static bool            s_modbus_ok  = false;

// =============================================================================
//  RS485 direction-pin callbacks (required by ModbusMaster)
// =============================================================================

static void rs485PreTransmit(void) {
    digitalWrite(PIN_RS485_DE, HIGH);
    delayMicroseconds(400);
}

static void rs485PostTransmit(void) {
    delayMicroseconds(400);
    digitalWrite(PIN_RS485_DE, LOW);
}

// =============================================================================
//  Helpers (private)
// =============================================================================

// Converts raw ModbusMaster result byte to a short diagnostic tag.
// Returned pointer is either a string literal or a static char buf —
// consume it before the next call.
static const char* modbusTag(uint8_t result) {
    if (result == ModbusMaster::ku8MBSuccess)             { return "OK";            }
    if (result == ModbusMaster::ku8MBResponseTimedOut)    { return "TO";            }
    if (result == ModbusMaster::ku8MBInvalidCRC)          { return "CRC";           }
    if (result == ModbusMaster::ku8MBInvalidSlaveID)      { return "BAD_SLAVE";     }
    if (result == ModbusMaster::ku8MBInvalidFunction)     { return "BAD_FN";        }
    if (result == ModbusMaster::ku8MBIllegalFunction)     { return "EX_ILLEGAL_FN"; }
    if (result == ModbusMaster::ku8MBIllegalDataAddress)  { return "EX_BAD_ADDR";   }
    if (result == ModbusMaster::ku8MBIllegalDataValue)    { return "EX_BAD_VAL";    }
    if (result == ModbusMaster::ku8MBSlaveDeviceFailure)  { return "EX_SLAVE_FAIL"; }
    static char s_buf[8];
    snprintf(s_buf, sizeof(s_buf), "0x%02X", result);
    return s_buf;
}

// Plausibility bounds — same as the original sketch.
// Tighten once you have known-good field readings.
static bool primaryInRange(float ec_uScm, float ph, float n_mgkg) {
    if (ph      <     0.0f || ph      >    14.0f)  { return false; }
    if (ec_uScm <     0.0f || ec_uScm > 200000.0f) { return false; }
    if (n_mgkg  <     0.0f || n_mgkg  > 100000.0f) { return false; }
    return true;
}

static bool stemmaTempInRange(float temp_c) {
    return (temp_c >= -40.0f && temp_c <= 85.0f);
}

// Derives the DataQ string that gets written to the CSV.
// Priority: Modbus fail → range fail → STEMMA temp fail → OK.
static const char* deriveDataQ(bool modbus_ok, bool range_ok, bool stemma_temp_ok) {
    if (!modbus_ok)     { return "NA_MB";       }
    if (!range_ok)      { return "RANGE_BAD";   }
    if (!stemma_temp_ok){ return "STEMMA_TEMP"; }
    return "OK";
}

// =============================================================================
//  Initialisation
// =============================================================================

bool initSoilSensors(void) {
    s_stemma_ok = false;
    s_modbus_ok = false;

    // ── STEMMA (I2C) ──────────────────────────────────────────────────────────
    // Wire is already started by the main sketch; no Wire.begin() needed here.
    for (int retry = 0; retry < CFG_MAX_RETRIES; retry++) {
        if (g_stemma.begin(STEMMA_ADDR)) {
            s_stemma_ok = true;
#ifdef DEBUG
            Serial.println("STEMMA: OK");
#endif
            break;
        }
        delay(100);
    }

    if (!s_stemma_ok) {
        g_error_state = ERR_SOIL_STEMMA;
#ifdef DEBUG
        Serial.println("STEMMA: FAIL");
#endif
    }

    // ── Primary sensor — RS485 / Modbus ───────────────────────────────────────
    pinMode(PIN_RS485_DE, OUTPUT);
    digitalWrite(PIN_RS485_DE, LOW);

    Serial2.begin(MODBUS_BAUD, SERIAL_8N1, PIN_RS485_RX, PIN_RS485_TX);
    g_modbus.begin(MODBUS_SLAVE, Serial2);
    g_modbus.preTransmission(rs485PreTransmit);
    g_modbus.postTransmission(rs485PostTransmit);

    // Settle time — many Modbus devices need this after baud negotiation.
    // This is a known 750 ms blocking call; acceptable here because
    // initGroundStage() runs 15 minutes after landing.
    delay(750);

    // Probe the sensor with a single read to confirm comms.
    uint8_t probe = g_modbus.readHoldingRegisters(0x0002, 3);
    s_modbus_ok = (probe == ModbusMaster::ku8MBSuccess);

    if (!s_modbus_ok) {
        g_error_state = ERR_SOIL_MODBUS;
#ifdef DEBUG
        Serial.printf("Modbus probe: %s\n", modbusTag(probe));
#endif
    } else {
#ifdef DEBUG
        Serial.println("Modbus: OK");
#endif
    }

    // Return true only if both sensors came up.
    // Caller can inspect g_error_state and s_stemma_ok / s_modbus_ok
    // to decide whether to continue with partial data.
    return (s_stemma_ok && s_modbus_ok);
}

// =============================================================================
//  Per-read function  (call at ~1 Hz from groundLoop)
// =============================================================================

bool readSoilSensors(SoilSample* out) {
    if (!assertNotNull(out, ERR_NULL_PTR)) { return false; }

    // Zero the output so no field is ever uninitialised on a partial failure.
    out->timestamp     = (unsigned long)millis();
    out->moisture      = 0.0f;
    out->stemma_temp_c = 0.0f;
    out->ec_uScm       = 0.0f;
    out->ph            = 0.0f;
    out->n_mgkg        = 0.0f;
    out->modbus_ok     = false;
    out->range_ok      = false;
    out->modbus_tag[0] = '\0';
    out->dataq[0]      = '\0';

    // ── STEMMA ────────────────────────────────────────────────────────────────
    if (s_stemma_ok) {
        uint16_t raw     = g_stemma.touchRead(0);
        float    moist   = ((float)raw - 200.0f) / 1800.0f;
        if (moist < 0.0f) { moist = 0.0f; }
        if (moist > 1.0f) { moist = 1.0f; }
        out->moisture      = moist;
        out->stemma_temp_c = g_stemma.getTemp();
    }

    // ── Primary (Modbus) ──────────────────────────────────────────────────────
    uint8_t result = g_modbus.readHoldingRegisters(0x0002, 3);
    out->modbus_ok = (result == ModbusMaster::ku8MBSuccess);

    // Copy the tag into the fixed-size field so the struct is self-contained.
    const char* tag = modbusTag(result);
    strncpy(out->modbus_tag, tag, sizeof(out->modbus_tag) - 1U);
    out->modbus_tag[sizeof(out->modbus_tag) - 1U] = '\0';

    if (out->modbus_ok) {
        out->ec_uScm = (float)g_modbus.getResponseBuffer(0);
        out->ph      = (float)g_modbus.getResponseBuffer(1) * 0.1f;
        out->n_mgkg  = (float)g_modbus.getResponseBuffer(2);
        out->range_ok = primaryInRange(out->ec_uScm, out->ph, out->n_mgkg);
    }

    // ── DataQ ─────────────────────────────────────────────────────────────────
    bool stemma_temp_ok = stemmaTempInRange(out->stemma_temp_c);
    const char* dq = deriveDataQ(out->modbus_ok, out->range_ok, stemma_temp_ok);
    strncpy(out->dataq, dq, sizeof(out->dataq) - 1U);
    out->dataq[sizeof(out->dataq) - 1U] = '\0';

#ifdef DEBUG
    Serial.printf("Soil ts=%lu  moist=%.3f  temp=%.1f  EC=%.0f  pH=%.1f  N=%.0f  MB=%s  Q=%s\n",
                  out->timestamp,
                  out->moisture, out->stemma_temp_c,
                  out->ec_uScm, out->ph, out->n_mgkg,
                  out->modbus_tag, out->dataq);
#endif

    // Return true if we got at least something useful.
    // A false return means both sensors failed completely.
    return (s_stemma_ok || out->modbus_ok);
}