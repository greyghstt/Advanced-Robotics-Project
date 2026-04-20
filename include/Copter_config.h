//PWM
#define PWM_MIN 988
#define PWM1_MIN 1047
#define PWM2_MIN 1047
#define PWM3_MIN 1047
#define PWM4_MIN 1047
#define PWM_MAX 2012

#define max_roll_corr 512//511
#define max_pitch_corr 512//513
#define max_yaw_corr 512//510
#define min_roll_corr -512//-514
#define min_pitch_corr -512//-510
#define min_yaw_corr -512//-517

//fbwastate
#define max_roll_limit 30
#define min_roll_limit -30
#define max_pitch_limit 30
#define min_pitch_limit -30
#define max_yaw_limit 20
#define min_yaw_limit -20

/** SERVO MAPPING */
#ifndef THROTTLE_MIN
#define THROTTLE_MIN 0 // percent
#endif
#ifndef THROTTLE_MAX
#define THROTTLE_MAX 100
#endif
#ifndef MIN_THROTTLE_PWM
#define MIN_THROTTLE_PWM 1000 //PWM
#endif
#ifndef MAX_THROTTLE_PWM
#define MAX_THROTTLE_PWM 2000
#endif

#define M_CONST 1.25

// Motor layout X/diagonal, dilihat dari atas:
// M1 GPIO33 = depan-kiri, CW
// M2 GPIO25 = depan-kanan, CCW
// M3 GPIO26 = belakang-kanan, CW
// M4 GPIO27 = belakang-kiri, CCW
//
// Roll memakai pasangan kiri (M1/M4) melawan kanan (M2/M3).
// Pitch memakai pasangan depan (M1/M2) melawan belakang (M3/M4).
// Yaw memakai pasangan CW (M1/M3) melawan CCW (M2/M4).
#define MOTOR_1 33
#define MOTOR_2 25
#define MOTOR_3 26
#define MOTOR_4 27
