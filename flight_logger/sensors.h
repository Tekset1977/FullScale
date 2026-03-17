#pragma once
#include "config.h"
#include "globals.h"

// -----------------------------------------------------------------------------
//  Initialisation
// -----------------------------------------------------------------------------
bool  initIMU(void);
bool  initBarometer(void);

// -----------------------------------------------------------------------------
//  Per-loop reads
// -----------------------------------------------------------------------------
bool  readIMUSensors(SensorData* data);
bool  pollBarometer(SensorData* data);
float baroReadAltitude(void);       // one-shot read used in setup() to zero baseline

// -----------------------------------------------------------------------------
//  Utilities
// -----------------------------------------------------------------------------
bool  detectMovement(float ax, float ay, float az);
void  resetI2CBus(void);
