#pragma once
#include "config.h"
#include "globals.h"

// -----------------------------------------------------------------------------
//  Initialisation
// -----------------------------------------------------------------------------
void storageBeginSPI(void);         // call once in setup() before initSD()
bool initSD(void);
bool initLogFile(void);

// -----------------------------------------------------------------------------
//  Per-loop writes
// -----------------------------------------------------------------------------
bool writeLogData(const SensorData* data);
bool writeServoEvent(float altitude_m);
void drainBufsToLog(void);
void flushLogFile(void);

// -----------------------------------------------------------------------------
//  Sleep / wake helpers (called by sleep.cpp)
// -----------------------------------------------------------------------------
void storageCloseFiles(void);
void storageReopenFiles(void);
