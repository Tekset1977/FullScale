#pragma once
#include "config.h"
#include "globals.h"

struct PidState {
    float kp, ki, kd;
    float integral;
    float prev_error;
    unsigned long last_ms;
};

void  initStabilizer(void);
void  updateStabilizer(const SensorData* data);
void  resetStabilizer(void);