#pragma once

#include "Arduino.h"
#include "Control_Modes.h"
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

void Transition_sequence_manual() {
    
    // Transition sequence for manual mode
    if (mode_now == 1) {
    copter_ControlFSFB(ch_roll, ch_pitch, ch_yaw, ch_throttle, roll, pitch, yaw, gx, gy, gz);
    copter_calcOutput(ch_throttle);
    motor_loop(m1_pwm,m2_pwm,m3_pwm,m4_pwm);
    }
}