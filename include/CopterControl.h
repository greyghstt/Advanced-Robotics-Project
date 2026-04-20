#ifndef COPTER_CONTROL_H
#define COPTER_CONTROL_H

#include <Arduino.h>

#include "ImuAcquisition.h"
#include "CopterConfig.h"
#include "Radio.h"
#include "Ultrasonic.h"
#include "ControlModes.h"

/// Quad-X mixer for the current wiring, viewed from above:
///
///            FRONT
///      M1 GPIO33 CW     M2 GPIO25 CCW
///
///      M4 GPIO27 CCW    M3 GPIO26 CW
///            REAR
///
/// Matrix columns are [throttle, roll, pitch, yaw].
/// Roll compares M1/M4 against M2/M3.
/// Pitch compares M1/M2 against M3/M4.
/// Yaw compares CW motors M1/M3 against CCW motors M2/M4.
///
/// BNO055 Euler output is x=heading/yaw, y=roll, z=pitch.
///
/// Reference only, not active:
/// This normalized 45-degree Quad-X mixer keeps the same sign convention but
/// uses the full projected arm coefficient. It is kept here as a reference if
/// the current field-tested matrix needs to be compared later.
// const double MIX_THRUST_COEFF = 292600.0;
// const double MIX_ROLL_PITCH_COEFF = 1838900.0;  // 2600600 * sin/cos(45 deg)
// const double MIX_YAW_COEFF = 6283300.0;
// const double A_invers[4][4] = {
//     {MIX_THRUST_COEFF,  MIX_ROLL_PITCH_COEFF,  MIX_ROLL_PITCH_COEFF, -MIX_YAW_COEFF},
//     {MIX_THRUST_COEFF, -MIX_ROLL_PITCH_COEFF,  MIX_ROLL_PITCH_COEFF,  MIX_YAW_COEFF},
//     {MIX_THRUST_COEFF, -MIX_ROLL_PITCH_COEFF, -MIX_ROLL_PITCH_COEFF, -MIX_YAW_COEFF},
//     {MIX_THRUST_COEFF,  MIX_ROLL_PITCH_COEFF, -MIX_ROLL_PITCH_COEFF,  MIX_YAW_COEFF}
// };
const double A_invers[4][4] = {
  {292600,  1300300,  1300300, -6283300},
  {292600,  -1300300, 1300300, 6283300},
  {292600, -1300300, -1300300, -6283300},
  {292600, 1300300,  -1300300, 6283300}
};

// Reference only, not active:
// Older vehicle-specific mixer kept for comparison with previous experiments.
// const double A_invers[4][4] = {
//     {-91600,  172800,  172800,  1471600},
//     {-91600,  172800, -172800, -1471600},
//     {-91600, -172800, -172800,  1471600},
//     {-91600, -172800,  172800, -1471600}
// };

    extern float gx, gy, gz;
    extern float delta_yaw, prev_yaw;
    float u1, u2, u3, u4;
    float w1, w2, w3, w4;
    float roll_int, pitch_int;
    unsigned long tnow, tbefore;
    float tdelta;
    unsigned long calc_time, last_calc_time;
    float delta_calc_time;
    float alt_ref, heading_now, last_alt, last_heading, alt_now;
    float alt_target, z_velocity;
    float roll_cmd, pitch_cmd, yaw_cmd;
    float min_roll = -20.0f;
    float max_roll = 20.0f;
    float min_pitch = -20.0f;
    float max_pitch = 20.0f;
    float min_yaw = -30.0f;
    float max_yaw = 30.0f;
    int PID_max_roll = 400;
    int PID_max_pitch = 400;
    int PID_max_yaw = 400;
    int PID_min_roll = -400;
    int PID_min_pitch = -400;
    int PID_min_yaw = -400;
    float trim_roll = 0.0f;
    float trim_pitch = 0.0f;
    float trim_yaw = 0.0f;
    float omega2[4];

    // Reference only, not active:
    // Older trim experiments. Current V1 baseline uses zero trim and relies on
    // mechanical setup plus PID tuning.
    // float trim_roll = 3.0f;
    // float trim_pitch = 3.5f;
    // float trim_yaw = -0.015f;

    // Reference only, not active:
    // Earlier high-gain experiment. The current V1 flight baseline uses much
    // smaller dashboard-friendly gains below.
    // struct gains {
    //     float k_alt = 2.0f;
    //     float k_z_velocity = 3.0f;
    //     float k_roll = 238.9f;
    //     float k_pitch = 195.0f;
    //     float k_yaw = 960.0f;
    //     float k_z_vel = 1.0f;
    //     float k_roll_rate = 5725.8f;
    //     float k_pitch_rate = 3995.8f;
    //     float k_yaw_rate = 0.0f;
    //     float k_i_roll = 0.0f;
    //     float k_i_pitch = 0.022f;
    //     float k_i_yaw = 0.05f;
    //     int16_t roll_rmt;
    //     int16_t pitch_rmt;
    //     int16_t yaw_rmt;
    // };
    // gains gain;

    struct gains {
        float k_alt             = 2.0;    
        float k_z_velocity      = 3.0;     
        float k_roll            = 3.5;    // V1 flight baseline
        float k_pitch           = 3.5;    // V1 flight baseline
        float k_yaw             = 4.0;    // V1 flight baseline
        float k_z_vel           = 1.0;     
        float k_roll_rate       = 1.3;    // V1 flight baseline
        float k_pitch_rate      = 1.3;    // V1 flight baseline
        float k_yaw_rate        = 0.5;    // V1 flight baseline
        float k_i_roll          = 0.3;    // V1 flight baseline
        float k_i_pitch         = 0.3;    // V1 flight baseline
        float k_i_yaw           = 0.0;    // V1 flight baseline
        int16_t roll_rmt;
        int16_t pitch_rmt;
        int16_t yaw_rmt;
    };
    gains gain;

    void copter_getIntegral(int16_t ch_thr, float roll, float pitch, float yaw) {
        tbefore = tnow;
        tnow = millis();
        tdelta = (tnow - tbefore) / 1000.0;
        roll_int += gain.k_i_roll * roll * tdelta;
        pitch_int += gain.k_i_pitch * pitch * tdelta;
        roll_int = constrain(roll_int, -15, 15);
        pitch_int = constrain(pitch_int, -15, 15);
        // Reset the integrators
        if (ch_thr < 1100) {
            roll_int = 0.0;
            pitch_int = 0.0;
        }
    }
    void copter_ControlFSFB(int16_t ch_r, int16_t ch_p, int16_t ch_y, int16_t ch_thr, float roll, float pitch, float yaw) {
        last_calc_time = calc_time;
        calc_time = micros();
        delta_calc_time = (calc_time - last_calc_time) / 1000000.0;
        heading_now = yaw;
        // Reference only, not active:
        // Draft altitude-hold logic using the ultrasonic sensor. It is kept as
        // a future starting point, but the current V1 flight baseline does not
        // use altitude hold in the control loop.
        // alt_ref = read_altitude();
        // alt_now = read_altitude();
        // if (alt_hold && mode_fbwa) {
        //     if (ch_throttle > 1600) {
        //         alt_setpoint = read_altitude();
        //         alt_setpoint += (ch_throttle - 1600) / 20000;
        //     } else if (ch_throttle < 1400) {
        //         alt_setpoint = read_altitude();
        //         alt_setpoint -= (ch_throttle - 1400) / 20000;
        //     } else {
        //         alt_setpoint = read_altitude();
        //     }
        //     alt_target = read_altitude() - alt_setpoint;
        //     z_velocity = (alt_now - last_alt) / delta_calc_time;
        // } else if (mode_fbwa && !alt_hold) {
        //     alt_target = read_altitude() - alt_ref;
        //     z_velocity = (read_altitude() - last_alt) / delta_calc_time;
        // }

        // Known V1 yaw note:
        // Current V1 yaw control uses delta_yaw as yaw_setpoint, so it behaves more like
        // yaw damping than true heading hold. This branch keeps that flown baseline
        // unchanged on purpose. If yaw remains unstable, tune yaw with moderate
        // k_yaw, stronger k_yaw_rate, and little/no k_i_yaw first.
        //
        // Commented options below are kept only as future reference.
        //
        // Cleaner future option for manual flight:
        //   yaw_rate_target = yaw_cmd;
        //   yaw_rate_error = yaw_rate_target - gz;
        //   u4 = gain.k_yaw_rate * yaw_rate_error;
        //
        // Later heading-hold option:
        //   keep yaw_target while yaw stick is centered,
        //   yaw_error = wrap180(yaw_target - yaw),
        //   u4 = gain.k_yaw * yaw_error - gain.k_yaw_rate * gz;
        //
        yaw_setpoint = delta_yaw;
        if (yaw_setpoint > 180) yaw_setpoint -= 360;
        if (yaw_setpoint < -180) yaw_setpoint += 360;
        copter_getIntegral(ch_thr, roll, pitch, yaw);

        roll_cmd = (map(ch_r - 1500, min_roll_corr, max_roll_corr, min_roll, max_roll));
        pitch_cmd = (map(ch_p - 1500, min_pitch_corr, max_pitch_corr, min_pitch, max_pitch));
        yaw_cmd = (map(ch_y - 1500, min_yaw_corr, max_yaw_corr, min_yaw, max_yaw));
        u1 = 0.0f;
        u2 = ((-gain.k_roll * ((-roll) + roll_int - (roll_cmd + trim_roll)) / 10000000.0f) + (-gain.k_roll_rate * (gy) / 10000000.0f));
        u3 = (gain.k_pitch * ((pitch) + pitch_int - (pitch_cmd + trim_pitch)) / 10000000.0f) + (-gain.k_pitch_rate * (gx) / 10000000.0f);
        u4 = ((-gain.k_yaw * (yaw_setpoint - (yaw_cmd + trim_yaw)) / 10000000.0f) + (gain.k_yaw_rate * (gz) / 10000000.0f));
        omega2[0] = (A_invers[0][0] * u1 + A_invers[0][1] * u2 + A_invers[0][2] * u3 + A_invers[0][3] * u4);
        omega2[1] = (A_invers[1][0] * u1 + A_invers[1][1] * u2 + A_invers[1][2] * u3 + A_invers[1][3] * u4);
        omega2[2] = (A_invers[2][0] * u1 + A_invers[2][1] * u2 + A_invers[2][2] * u3 + A_invers[2][3] * u4);
        omega2[3] = (A_invers[3][0] * u1 + A_invers[3][1] * u2 + A_invers[3][2] * u3 + A_invers[3][3] * u4);
        last_heading = heading_now;
        
    }
#endif // COPTER_CONTROL_H
