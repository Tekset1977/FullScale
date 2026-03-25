#pragma once
#include "config.h"
#include "globals.h"

// -----------------------------------------------------------------------------
//  SoilSample — filled by readSoilSensors(), written to SD by writeSoilData()
// -----------------------------------------------------------------------------
struct SoilSample {
    unsigned long timestamp;
    float         moisture;        // 0.0–1.0  (STEMMA normalised)
    float         stemma_temp_c;   // degrees C (STEMMA)
    float         ec_uScm;         // uS/cm  (Modbus reg 0)
    float         ph;              // 0–14   (Modbus reg 1, scaled x0.1)
    float         n_mgkg;          // mg/kg  (Modbus reg 2)
    bool          modbus_ok;       // true if Modbus read succeeded
    bool          range_ok;        // true if EC/pH/N passed plausibility check
    char          modbus_tag[16];  // FIX 1: was missing — short diagnostic e.g. "OK","TO","CRC"
    char          dataq[16];       // FIX 1: was missing — "OK"|"NA_MB"|"RANGE_BAD"|"STEMMA_TEMP"
};

// -----------------------------------------------------------------------------
//  API
// -----------------------------------------------------------------------------
bool initSoilSensors(void);
bool readSoilSensors(SoilSample* out);