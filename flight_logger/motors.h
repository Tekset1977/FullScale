#pragma once
#include "config.h"
#include "globals.h"

bool  initMotors(void);
void  setMotor(uint8_t channel, int speed);   // channel 0/1, speed -255..255
void  stopMotors(void);