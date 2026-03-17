#pragma once
#include "config.h"
#include "globals.h"

bool initDisplay(void);
void updateDisplay(const SensorData* data, bool moving, unsigned long idle_ms);
void displaySensorFail(bool imu_ok, bool baro_ok);
void displayEnterSleep(void);
