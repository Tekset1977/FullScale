#pragma once
#include "config.h"
#include "globals.h"
#include "soil.h"    // FIX 2: was missing — needed for SoilSample* in writeSoilData()

// -----------------------------------------------------------------------------
//  Initialisation
// -----------------------------------------------------------------------------
void storageBeginSPI(void);
bool initSD(void);
bool initLogFile(void);
bool initSoilLogFile(void);

// -----------------------------------------------------------------------------
//  Per-loop writes
// -----------------------------------------------------------------------------
bool writeLogData(const SensorData* data);
bool writeServoEvent(float altitude_m);
bool writeSoilData(const SoilSample* s);
void drainBufsToLog(void);
void flushLogFile(void);

// -----------------------------------------------------------------------------
//  Sleep / wake helpers (called by sleep.cpp)
// -----------------------------------------------------------------------------
void storageCloseFiles(void);
void storageReopenFiles(void);