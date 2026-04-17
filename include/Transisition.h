#pragma once

#include "Arduino.h"
#include "Control_modes.h"
#include "Radio.h"
#include "Copter_control.h"
#include "Actuator.h"

float mode_phase, mode_fw;
float decrease = 0.0f;  // ignore
float true_aileron, true_elevator, true_rudder;
bool takeoff_is_done = false;
bool landing_is_done = false;
bool init_time = true;
bool start_landing = false;
bool break_start = false;
bool break_is_done = false;
bool transition_is_done = false;
uint8_t break_delay;
uint8_t time_flag;
float control_signal;
float init_pwm_tkff = 1100;
float init_pwm_land;
float k_p = 0.0;
float k_i = 0.0;
float k_d = 0.0;
int autonav_state = 0;  /// variable to check auto-loop state, delete after testing
bool enterauto = false;
int32_t temp_alt = 0;

// Safety gate: arm hanya mengizinkan kontrol, motor baru boleh naik setelah throttle melewati ambang ini.
#define MOTOR_START_THROTTLE 1100
#define RADIO_TIMEOUT_MS 150

void stop_motors_safe() {
    roll_int = 0.0f;
    pitch_int = 0.0f;
    omega2[0] = 0.0f;
    omega2[1] = 0.0f;
    omega2[2] = 0.0f;
    omega2[3] = 0.0f;
    control1 = 0;
    control2 = 0;
    control3 = 0;
    control4 = 0;
    motor_loop(servo_min, servo_min, servo_min, servo_min);
}

void Transition_sequence_manual() {
    bool radio_timeout = radio_frame_valid && ((millis() - last_radio_frame_ms) > RADIO_TIMEOUT_MS);

    if (!radio_frame_valid || signal_lost || radio_failsafe || radio_timeout) {
        arming = false;
        stop_motors_safe();
        return;
    }

    if (!arming || ch_throttle < MOTOR_START_THROTTLE) {
        stop_motors_safe();
        return;
    }

#if ESC_DIRECT_THROTTLE_TEST
    motor_loop(ch_throttle, ch_throttle, ch_throttle, ch_throttle);
    return;
#endif

    copter_ControlFSFB(ch_roll, ch_pitch, ch_yaw, ch_throttle, roll, pitch, yaw);
    copter_calcOutput(ch_throttle);
    motor_loop(m1_pwm, m2_pwm, m3_pwm, m4_pwm);
}
