#ifndef COPTER_CONTROL_H
#define COPTER_CONTROL_H

#include <Arduino.h>
#include <math.h>

#include "FlightConfig.h"
#include "Radio.h"
#include "ControlModes.h"

/*
 * V4 controller concept:
 * - pilot roll/pitch stick -> target angle
 * - angle error -> target body rate
 * - body-rate error + small optional integral -> motor correction in PWM us
 * - direct Quad-X mixer -> omega2 compatibility output for Actuator.h
 *
 * This keeps the old telemetry/dashboard variable names, but removes the old
 * inverse-matrix scaling so tuning values are easier to reason about.
 *
 * Motor layout used by this V4 branch, viewed from above:
 *
 *            FRONT
 *      M1 GPIO33 CW      M2 GPIO25 CCW
 *
 *      M4 GPIO27 CCW     M3 GPIO26 CW
 *            REAR
 *
 * Mixer groups:
 * - Roll  : left pair M1/M4 against right pair M2/M3.
 * - Pitch : front pair M1/M2 against rear pair M3/M4.
 * - Yaw   : CW pair M1/M3 against CCW pair M2/M4.
 */

static const float V4_MAX_ROLL_ANGLE_DEG = 25.0f;
static const float V4_MAX_PITCH_ANGLE_DEG = 25.0f;
static const float V4_MAX_YAW_RATE_DPS = 90.0f;
static const float V4_RC_DEADBAND_US = 12.0f;
static const float V4_INTEGRATOR_LIMIT_US = 80.0f;
static const float V4_AXIS_OUTPUT_LIMIT_US = 260.0f;
static const float V4_YAW_OUTPUT_LIMIT_US = 160.0f;
static const float V4_LOW_THROTTLE_RESET_US = 1120.0f;
static const float V4_TPA_BREAKPOINT_US = 1450.0f;
static const float V4_TPA_AMOUNT = 0.35f;
static const float V4_OUTPUT_SLEW_LIMIT_US_PER_LOOP = 80.0f;

/*
 * Sensor-axis signs. These are intentionally explicit because the BNO055
 * board orientation may not match the airframe axes.
 */
static const float V4_ROLL_ATTITUDE_SIGN = -1.0f;
static const float V4_PITCH_ATTITUDE_SIGN = 1.0f;
static const float V4_ROLL_RATE_SIGN = -1.0f;   // roll angle uses -roll, so rate follows -gy
static const float V4_PITCH_RATE_SIGN = 1.0f;   // pitch angle uses pitch, so rate follows gx
static const float V4_YAW_RATE_SIGN = 1.0f;

float u1, u2, u3, u4;
float w1, w2, w3, w4;
float roll_int, pitch_int, yaw_int;
unsigned long tnow, tbefore;
float tdelta;
unsigned long calc_time, last_calc_time;
float delta_calc_time;
float alt_ref, heading_now, last_alt, last_heading, alt_now;
float alt_target, z_velocity;
float roll_cmd, pitch_cmd, yaw_cmd;
float min_roll = -V4_MAX_ROLL_ANGLE_DEG;
float max_roll = V4_MAX_ROLL_ANGLE_DEG;
float min_pitch = -V4_MAX_PITCH_ANGLE_DEG;
float max_pitch = V4_MAX_PITCH_ANGLE_DEG;
float min_yaw = -V4_MAX_YAW_RATE_DPS;
float max_yaw = V4_MAX_YAW_RATE_DPS;
int PID_max_roll = (int)V4_AXIS_OUTPUT_LIMIT_US;
int PID_max_pitch = (int)V4_AXIS_OUTPUT_LIMIT_US;
int PID_max_yaw = (int)V4_YAW_OUTPUT_LIMIT_US;
int PID_min_roll = -(int)V4_AXIS_OUTPUT_LIMIT_US;
int PID_min_pitch = -(int)V4_AXIS_OUTPUT_LIMIT_US;
int PID_min_yaw = -(int)V4_YAW_OUTPUT_LIMIT_US;
float trim_roll = 0.0f;
float trim_pitch = 0.0f;
float trim_yaw = 0.0f;
float omega2[4];
float v4_last_correction[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float roll_out_debug = 0.0f;
float pitch_out_debug = 0.0f;
float yaw_out_debug = 0.0f;

struct gains {
    float k_alt = 2.0f;
    float k_z_velocity = 3.0f;
    float k_roll = 3.5f;       // angle P: deg error -> deg/s target
    float k_pitch = 3.5f;      // angle P: deg error -> deg/s target
    float k_yaw = 0.6f;        // yaw stick feed-forward into PWM correction
    float k_z_vel = 1.0f;
    float k_roll_rate = 1.0f;  // rate P/damping: deg/s error -> PWM correction
    float k_pitch_rate = 1.0f; // rate P/damping: deg/s error -> PWM correction
    float k_yaw_rate = 0.5f;   // yaw-rate damping
    float k_i_roll = 0.0f;     // keep I off for first V4 tuning pass
    float k_i_pitch = 0.0f;
    float k_i_yaw = 0.0f;
    int16_t roll_rmt;
    int16_t pitch_rmt;
    int16_t yaw_rmt;
};
gains gain;

float v4_clampf(float value, float low, float high) {
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

float v4_rc_normalized(int16_t pwm) {
    float centered = (float)pwm - 1500.0f;

    if (fabsf(centered) <= V4_RC_DEADBAND_US) {
        return 0.0f;
    }

    if (centered > 0.0f) {
        centered -= V4_RC_DEADBAND_US;
        return v4_clampf(centered / (500.0f - V4_RC_DEADBAND_US), 0.0f, 1.0f);
    }

    centered += V4_RC_DEADBAND_US;
    return v4_clampf(centered / (500.0f - V4_RC_DEADBAND_US), -1.0f, 0.0f);
}

float v4_tpa_scale(int16_t ch_thr) {
    if ((float)ch_thr <= V4_TPA_BREAKPOINT_US) {
        return 1.0f;
    }

    float progress = ((float)ch_thr - V4_TPA_BREAKPOINT_US) /
                     ((float)MAX_THROTTLE_PWM - V4_TPA_BREAKPOINT_US);
    progress = v4_clampf(progress, 0.0f, 1.0f);
    return 1.0f - (V4_TPA_AMOUNT * progress);
}

void copter_reset_control_state() {
    roll_int = 0.0f;
    pitch_int = 0.0f;
    yaw_int = 0.0f;
    u1 = 0.0f;
    u2 = 0.0f;
    u3 = 0.0f;
    u4 = 0.0f;
    roll_out_debug = 0.0f;
    pitch_out_debug = 0.0f;
    yaw_out_debug = 0.0f;
    omega2[0] = 0.0f;
    omega2[1] = 0.0f;
    omega2[2] = 0.0f;
    omega2[3] = 0.0f;
    v4_last_correction[0] = 0.0f;
    v4_last_correction[1] = 0.0f;
    v4_last_correction[2] = 0.0f;
    v4_last_correction[3] = 0.0f;
}

void copter_getIntegral(int16_t ch_thr, float roll_error, float pitch_error, float yaw_rate_error) {
    if (ch_thr < V4_LOW_THROTTLE_RESET_US) {
        copter_reset_control_state();
        return;
    }

    roll_int += gain.k_i_roll * roll_error * delta_calc_time;
    pitch_int += gain.k_i_pitch * pitch_error * delta_calc_time;
    yaw_int += gain.k_i_yaw * yaw_rate_error * delta_calc_time;

    roll_int = v4_clampf(roll_int, -V4_INTEGRATOR_LIMIT_US, V4_INTEGRATOR_LIMIT_US);
    pitch_int = v4_clampf(pitch_int, -V4_INTEGRATOR_LIMIT_US, V4_INTEGRATOR_LIMIT_US);
    yaw_int = v4_clampf(yaw_int, -V4_INTEGRATOR_LIMIT_US, V4_INTEGRATOR_LIMIT_US);
}

void v4_limit_motor_corrections(float correction[4], int16_t ch_thr) {
    float positive_headroom = (float)MAX_THROTTLE_PWM - (float)ch_thr;
    float negative_headroom = (float)ch_thr - (float)MIN_THROTTLE_PWM;
    float scale = 1.0f;

    for (uint8_t i = 0; i < 4; i++) {
        correction[i] = v4_clampf(correction[i], -V4_AXIS_OUTPUT_LIMIT_US, V4_AXIS_OUTPUT_LIMIT_US);

        if (correction[i] > positive_headroom && correction[i] > 0.0f) {
            scale = fminf(scale, positive_headroom / correction[i]);
        } else if (correction[i] < -negative_headroom && correction[i] < 0.0f) {
            scale = fminf(scale, negative_headroom / -correction[i]);
        }
    }

    scale = v4_clampf(scale, 0.0f, 1.0f);

    for (uint8_t i = 0; i < 4; i++) {
        correction[i] *= scale;
    }
}

void v4_slew_limit_corrections(float correction[4]) {
    for (uint8_t i = 0; i < 4; i++) {
        float delta = correction[i] - v4_last_correction[i];
        delta = v4_clampf(delta, -V4_OUTPUT_SLEW_LIMIT_US_PER_LOOP, V4_OUTPUT_SLEW_LIMIT_US_PER_LOOP);
        correction[i] = v4_last_correction[i] + delta;
        v4_last_correction[i] = correction[i];
    }
}

void copter_ControlFSFB(int16_t ch_r, int16_t ch_p, int16_t ch_y, int16_t ch_thr,
                        float roll, float pitch, float yaw, float gx, float gy, float gz) {
    last_calc_time = calc_time;
    calc_time = micros();
    delta_calc_time = (calc_time - last_calc_time) / 1000000.0f;

    if (last_calc_time == 0 || delta_calc_time <= 0.0f || delta_calc_time > 0.05f) {
        delta_calc_time = 0.005f;
    }

    heading_now = yaw;
    roll_cmd = v4_rc_normalized(ch_r) * V4_MAX_ROLL_ANGLE_DEG;
    pitch_cmd = v4_rc_normalized(ch_p) * V4_MAX_PITCH_ANGLE_DEG;
    yaw_cmd = v4_rc_normalized(ch_y) * V4_MAX_YAW_RATE_DPS;

    float roll_angle = (V4_ROLL_ATTITUDE_SIGN * roll);
    float pitch_angle = (V4_PITCH_ATTITUDE_SIGN * pitch);
    float roll_rate = (V4_ROLL_RATE_SIGN * gy);
    float pitch_rate = (V4_PITCH_RATE_SIGN * gx);
    float yaw_rate = (V4_YAW_RATE_SIGN * gz);

    float roll_error = (roll_cmd + trim_roll) - roll_angle;
    float pitch_error = (pitch_cmd + trim_pitch) - pitch_angle;
    float roll_rate_target = gain.k_roll * roll_error;
    float pitch_rate_target = gain.k_pitch * pitch_error;
    float roll_rate_error = roll_rate_target - roll_rate;
    float pitch_rate_error = pitch_rate_target - pitch_rate;
    float yaw_rate_target = yaw_cmd + trim_yaw;
    float yaw_rate_error = yaw_rate_target - yaw_rate;

    copter_getIntegral(ch_thr, roll_error, pitch_error, yaw_rate_error);

    float tpa = v4_tpa_scale(ch_thr);
    float roll_out = ((gain.k_roll_rate * roll_rate_error) + roll_int) * tpa;
    float pitch_out = ((gain.k_pitch_rate * pitch_rate_error) + pitch_int) * tpa;
    float yaw_out = ((gain.k_yaw * yaw_rate_target) + (-gain.k_yaw_rate * yaw_rate) + yaw_int) * tpa;

    // Temporarily disabled for open-loop tuning tests.
    // roll_out = v4_clampf(roll_out, -V4_AXIS_OUTPUT_LIMIT_US, V4_AXIS_OUTPUT_LIMIT_US);
    // pitch_out = v4_clampf(pitch_out, -V4_AXIS_OUTPUT_LIMIT_US, V4_AXIS_OUTPUT_LIMIT_US);
    // yaw_out = v4_clampf(yaw_out, -V4_YAW_OUTPUT_LIMIT_US, V4_YAW_OUTPUT_LIMIT_US);

    u1 = 0.0f;
    u2 = roll_out;
    u3 = pitch_out;
    u4 = yaw_out;
    roll_out_debug = roll_out;
    pitch_out_debug = pitch_out;
    yaw_out_debug = yaw_out;

    /*
     * Quad-X direct mixer for the configured motor layout:
     *
     *   M1 front-left  GPIO33 CW
     *   M2 front-right GPIO25 CCW
     *   M3 rear-right  GPIO26 CW
     *   M4 rear-left   GPIO27 CCW
     *
     * Signs below intentionally match the physical groups:
     *   +roll_out  -> M1/M4 up, M2/M3 down
     *   +pitch_out -> M3/M4 up, M1/M2 down
     *   +yaw_out   -> M2/M4 up, M1/M3 down
     */
    float correction[4] = {
        roll_out - pitch_out - yaw_out,
        -roll_out - pitch_out + yaw_out,
        -roll_out + pitch_out - yaw_out,
        roll_out + pitch_out + yaw_out
    };

    // Temporarily disabled for open-loop tuning tests.
    // v4_limit_motor_corrections(correction, ch_thr);
    // v4_slew_limit_corrections(correction);

    omega2[0] = correction[0] / M_CONST;
    omega2[1] = correction[1] / M_CONST;
    omega2[2] = correction[2] / M_CONST;
    omega2[3] = correction[3] / M_CONST;

    last_heading = heading_now;
}

#endif
