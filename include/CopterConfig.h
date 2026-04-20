// ESC PWM calibration.
#define PWM_MIN 988
#define PWM1_MIN 1047
#define PWM2_MIN 1047
#define PWM3_MIN 1047
#define PWM4_MIN 1047
#define PWM_MAX 2012

#define max_roll_corr 512
#define max_pitch_corr 512
#define max_yaw_corr 512
#define min_roll_corr -512
#define min_pitch_corr -512
#define min_yaw_corr -512

// Stick command limits.
#define max_roll_limit 30
#define min_roll_limit -30
#define max_pitch_limit 30
#define min_pitch_limit -30
#define max_yaw_limit 20
#define min_yaw_limit -20

// Throttle conversion limits.
#ifndef THROTTLE_MIN
#define THROTTLE_MIN 0 // percent
#endif
#ifndef THROTTLE_MAX
#define THROTTLE_MAX 100
#endif
#ifndef MIN_THROTTLE_PWM
#define MIN_THROTTLE_PWM 1000
#endif
#ifndef MAX_THROTTLE_PWM
#define MAX_THROTTLE_PWM 2000
#endif

#define M_CONST 1.25

// Quad-X motor layout, viewed from above:
// M1 GPIO33 = front-left, CW
// M2 GPIO25 = front-right, CCW
// M3 GPIO26 = rear-right, CW
// M4 GPIO27 = rear-left, CCW
//
// Roll compares left motors (M1/M4) against right motors (M2/M3).
// Pitch compares front motors (M1/M2) against rear motors (M3/M4).
// Yaw compares CW motors (M1/M3) against CCW motors (M2/M4).
#define MOTOR_1 33
#define MOTOR_2 25
#define MOTOR_3 26
#define MOTOR_4 27
