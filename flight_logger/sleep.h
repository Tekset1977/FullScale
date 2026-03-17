#pragma once
#include "config.h"
#include "globals.h"

void enterLightSleep(void);
void updateAltitudeIdleTimer(float altitude_m, unsigned long* wake_time_ms);
