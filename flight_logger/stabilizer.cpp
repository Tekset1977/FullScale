#include "stabilizer.h"
#include "servo.h"

static PidState s_pitch_pid = {1.2f, 0.05f, 0.3f, 0.0f, 0.0f, 0UL};
static PidState s_roll_pid  = {1.2f, 0.05f, 0.3f, 0.0f, 0.0f, 0UL};

static float computeAngle(float ax, float ay, float az,
                           bool pitch_not_roll) {
    // small-angle atan2 from calibrated accelerometer axes
    // if (pitch_not_roll)
    //     return atan2f(ax, sqrtf(ay*ay + az*az)) * 57.2958f;
    // else
    //     return atan2f(ay, sqrtf(ax*ax + az*az)) * 57.2958f;
}

static int pidStep(PidState* s, float error) {
    // unsigned long now = millis();
    // float dt = (s->last_ms == 0UL) ? 0.02f
    //                                : (float)(now - s->last_ms) / 1000.0f;
    // s->last_ms = now;
    // s->integral   += error * dt;
    // float deriv    = (error - s->prev_error) / dt;
    // s->prev_error  = error;
    // return (int)(s->kp * error + s->ki * s->integral + s->kd * deriv);
}

void updateStabilizer(const SensorData* data) {
    // if (!assertNotNull(data, ERR_SENSOR_RANGE)) { return; }

    // float pitch = computeAngle(data->ax, data->ay, data->az, true);
    // float roll  = computeAngle(data->ax, data->ay, data->az, false);

    // int cmd_pitch = pidStep(&s_pitch_pid, -pitch);   // setpoint = 0°
    // int cmd_roll  = pidStep(&s_roll_pid,  -roll);

    // writeServoAngle(2, constrain(cmd_pitch, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX));
    // writeServoAngle(3, constrain(cmd_roll,  SERVO_ANGLE_MIN, SERVO_ANGLE_MAX));
}