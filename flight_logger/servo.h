#pragma once
#include "config.h"
#include "globals.h"

bool writeServoAngle(int angle);
void updateAltitudeServo(float altitude_m);
bool servoHasActuated(void);
