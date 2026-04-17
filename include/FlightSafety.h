#pragma once

#include <Arduino.h>

#include "ControlModes.h"
#include "Radio.h"
#include "ImuSensor.h"
#include "FlightControl.h"
#include "Actuator.h"

static const int16_t MOTOR_START_THROTTLE = 1100;
static const int16_t PREARM_MAX_START_THROTTLE = 1200;
static const unsigned long RADIO_TIMEOUT_MS = 150;
static const unsigned long IMU_TIMEOUT_MS = 100;
static const float PREARM_MAX_TILT_DEG = 35.0f;

bool motors_started = false;
const char *flight_safety_reason = "boot";

void stop_motors_safe() {
    motors_started = false;
    copter_reset_control_state();
    control1 = 0;
    control2 = 0;
    control3 = 0;
    control4 = 0;
    motor_loop(servo_min, servo_min, servo_min, servo_min);
}

bool radio_timeout_active() {
    return radio_frame_valid && ((millis() - last_radio_frame_ms) > RADIO_TIMEOUT_MS);
}

bool flight_prearm_checks_pass() {
    if (!actuator_ready) {
        flight_safety_reason = "actuator";
        return false;
    }

    if (!imu_recent(IMU_TIMEOUT_MS)) {
        flight_safety_reason = "imu";
        return false;
    }

    if (fabsf(roll) > PREARM_MAX_TILT_DEG || fabsf(pitch) > PREARM_MAX_TILT_DEG) {
        flight_safety_reason = "tilt";
        return false;
    }

    if (ch_throttle > PREARM_MAX_START_THROTTLE) {
        flight_safety_reason = "throttle";
        return false;
    }

    flight_safety_reason = "ok";
    return true;
}

void flight_safety_update() {
    if (!radio_frame_valid || signal_lost || radio_failsafe || radio_timeout_active()) {
        arming = false;
        flight_safety_reason = "radio";
        stop_motors_safe();
        return;
    }

    if (!arming) {
        flight_safety_reason = "disarmed";
        stop_motors_safe();
        return;
    }

    if (ch_throttle < MOTOR_START_THROTTLE) {
        flight_safety_reason = "idle";
        stop_motors_safe();
        return;
    }

    if (!motors_started && !flight_prearm_checks_pass()) {
        stop_motors_safe();
        return;
    }

    motors_started = true;

#if ESC_DIRECT_THROTTLE_TEST
    motor_loop(ch_throttle, ch_throttle, ch_throttle, ch_throttle);
    return;
#endif

    copter_ControlFSFB(ch_roll, ch_pitch, ch_yaw, ch_throttle, roll, pitch, yaw, gx, gy, gz);
    copter_calcOutput(ch_throttle);
    motor_loop(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
}
