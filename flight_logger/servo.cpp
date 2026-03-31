#include "servo.h"
#include "storage.h"   // for writeServoEvent

// =============================================================================
//  PWM helpers (private)
// =============================================================================

static uint32_t servoPeriodUs(void) {
    if (PWM_FREQ_HZ == 0UL) { return 0UL; }
    uint32_t period = 1000000UL / PWM_FREQ_HZ;
    if (period < 300UL || period > 20000UL) { return 0UL; }
    return period;
}

static uint32_t usToDuty(uint32_t pulse_us) {
    uint32_t period = servoPeriodUs();
    if (period == 0UL) { return 0UL; }
    if (pulse_us > period) { pulse_us = period; }

    uint32_t max_duty = (1UL << PWM_RES_BITS) - 1UL;
    uint32_t duty = (pulse_us * max_duty) / period;
    if (duty > max_duty) { duty = max_duty; }
    return duty;
}

static bool writeServoPulse(uint32_t pulse_us) {
    uint32_t duty     = usToDuty(pulse_us);
    uint32_t max_duty = (1UL << PWM_RES_BITS) - 1UL;
    if (duty > max_duty) { return false; }
    return ledcWrite(PIN_SERVO, duty);
}

static bool g_servo_actuated = false;

// =============================================================================
//  Public API
// =============================================================================

bool writeServoAngle(int angle) {
    if (angle < SERVO_ANGLE_MIN - 10 || angle > SERVO_ANGLE_MAX + 10) {
        return false;
    }
    int clamped = constrain(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
    long pulse  = map(clamped, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX,
                      (long)PULSE_MIN_US, (long)PULSE_MAX_US);

    if ((uint32_t)pulse < PULSE_MIN_US || (uint32_t)pulse > PULSE_MAX_US) {
        return false;
    }
    return writeServoPulse((uint32_t)pulse);
}

// Altitude-band release — triggers once per flight when descending
// through the configured band after having climbed above it.
void updateAltitudeServo(float altitude_m) {
    static bool     s_above_band = false;
    static bool     s_triggered  = false;
    static bool     s_moving     = false;
    static uint32_t s_move_start = 0UL;

    if (!assertRange(altitude_m, ALT_MIN_M, ALT_MAX_M, ERR_SENSOR_RANGE)) { return; }
    if (SERVO_ANGLE_INIT < SERVO_ANGLE_MIN || SERVO_ANGLE_INIT > SERVO_ANGLE_MAX) { return; }
    if (SERVO_ANGLE_GO  < SERVO_ANGLE_MIN || SERVO_ANGLE_GO  > SERVO_ANGLE_MAX)  { return; }

    if (altitude_m > ALT_BAND_HI_M) {
        s_above_band = true;
    }

    if (s_above_band && !s_triggered && altitude_m < ALT_BAND_LO_M) {
        s_triggered  = true;
        s_moving     = true;
        s_move_start = (uint32_t)millis();

        bool ev_ok = writeServoEvent(altitude_m);
        bool ok    = writeServoAngle(SERVO_ANGLE_GO);
        if (ok) {
            g_servo_actuated = true;
        }

#ifdef DEBUG
        Serial.printf("SERVO TRIGGER — ts=%lu ms  alt=%.3f m\n",
                      (unsigned long)millis(), altitude_m);
        if (!ev_ok) { Serial.println("Servo event log FAILED"); }
        if (!ok)    { Serial.println("Servo write failed on release"); }
#endif
    }

    // Checked every call, outside the trigger block
    if (s_moving && ((uint32_t)millis() - s_move_start) >= SERVO_MOVE_MS) {
        s_moving = false;
    }
}

bool servoHasActuated(void) {
    return g_servo_actuated;
}
