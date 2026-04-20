#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h>
#include "FlightControl.h"

#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

// Quad-X motor layout, viewed from above:
//            FRONT
//      M1 GPIO33 CW      M2 GPIO25 CCW
//
//      M4 GPIO27 CCW     M3 GPIO26 CW
//            REAR
#define MOTOR_1_PIN  33
#define MOTOR_2_PIN  25
#define MOTOR_3_PIN  26
#define MOTOR_4_PIN  27

#define ESC_DIRECT_THROTTLE_TEST 0
#define ESC_PWM_HZ 50
#define ESC_PWM_RESOLUTION_BITS 16
#define ESC_PWM_MAX_DUTY ((1UL << ESC_PWM_RESOLUTION_BITS) - 1UL)
#define CONTROL_OUTPUT_LIMIT_US 400

#define MOTOR_1_CHANNEL 0
#define MOTOR_2_CHANNEL 1
#define MOTOR_3_CHANNEL 2
#define MOTOR_4_CHANNEL 3

int control1, control2, control3, control4;
float m1_pwm, m2_pwm, m3_pwm, m4_pwm;
uint16_t servo_max = MAX_THROTTLE_PWM;
uint16_t servo_min = MIN_THROTTLE_PWM;
uint16_t throttle;
bool actuator_ready = false;

uint32_t esc_pulse_to_duty(int pwm_us) {
    pwm_us = constrain(pwm_us, servo_min, servo_max);
    const uint32_t period_us = 1000000UL / ESC_PWM_HZ;
    return ((uint32_t)pwm_us * ESC_PWM_MAX_DUTY) / period_us;
}

void write_esc_us(uint8_t channel, int pwm_us) {
    ledcWrite(channel, esc_pulse_to_duty(pwm_us));
}

void motor_loop(int pwm1, int pwm2, int pwm3, int pwm4);

void init_actuator() {
    ledcSetup(MOTOR_1_CHANNEL, ESC_PWM_HZ, ESC_PWM_RESOLUTION_BITS);
    ledcSetup(MOTOR_2_CHANNEL, ESC_PWM_HZ, ESC_PWM_RESOLUTION_BITS);
    ledcSetup(MOTOR_3_CHANNEL, ESC_PWM_HZ, ESC_PWM_RESOLUTION_BITS);
    ledcSetup(MOTOR_4_CHANNEL, ESC_PWM_HZ, ESC_PWM_RESOLUTION_BITS);

    ledcAttachPin(MOTOR_1_PIN, MOTOR_1_CHANNEL);
    ledcAttachPin(MOTOR_2_PIN, MOTOR_2_CHANNEL);
    ledcAttachPin(MOTOR_3_PIN, MOTOR_3_CHANNEL);
    ledcAttachPin(MOTOR_4_PIN, MOTOR_4_CHANNEL);

    actuator_ready = true;
    motor_loop(servo_min, servo_min, servo_min, servo_min);

    Serial.println("Actuator LEDC setup complete");
}

void copter_calcOutput(int16_t ch_thr) {
    throttle = constrain(ch_thr, servo_min, servo_max);

    control1 = (int)lroundf(omega2[0] * M_CONST);
    control2 = (int)lroundf(omega2[1] * M_CONST);
    control3 = (int)lroundf(omega2[2] * M_CONST);
    control4 = (int)lroundf(omega2[3] * M_CONST);

    control1 = constrain(control1, -CONTROL_OUTPUT_LIMIT_US, CONTROL_OUTPUT_LIMIT_US);
    control2 = constrain(control2, -CONTROL_OUTPUT_LIMIT_US, CONTROL_OUTPUT_LIMIT_US);
    control3 = constrain(control3, -CONTROL_OUTPUT_LIMIT_US, CONTROL_OUTPUT_LIMIT_US);
    control4 = constrain(control4, -CONTROL_OUTPUT_LIMIT_US, CONTROL_OUTPUT_LIMIT_US);

    if (arming) {
        m1_pwm = control1 + throttle;
        m2_pwm = control2 + throttle;
        m3_pwm = control3 + throttle;
        m4_pwm = control4 + throttle;
    } else {
        m1_pwm = servo_min;
        m2_pwm = servo_min;
        m3_pwm = servo_min;
        m4_pwm = servo_min;
    }

    m1_pwm = constrain(m1_pwm, servo_min, servo_max);
    m2_pwm = constrain(m2_pwm, servo_min, servo_max);
    m3_pwm = constrain(m3_pwm, servo_min, servo_max);
    m4_pwm = constrain(m4_pwm, servo_min, servo_max);
}

void motor_loop(int pwm1, int pwm2, int pwm3, int pwm4) {
    pwm1 = constrain(pwm1, servo_min, servo_max);
    pwm2 = constrain(pwm2, servo_min, servo_max);
    pwm3 = constrain(pwm3, servo_min, servo_max);
    pwm4 = constrain(pwm4, servo_min, servo_max);

    m1_pwm = pwm1;
    m2_pwm = pwm2;
    m3_pwm = pwm3;
    m4_pwm = pwm4;

    if (!actuator_ready) {
        return;
    }

    write_esc_us(MOTOR_1_CHANNEL, pwm1);
    write_esc_us(MOTOR_2_CHANNEL, pwm2);
    write_esc_us(MOTOR_3_CHANNEL, pwm3);
    write_esc_us(MOTOR_4_CHANNEL, pwm4);
}

float PercenttoPWM(float percent) {
    return map(percent, 0, 100, servo_min, servo_max);
}

#endif // ACTUATOR_H
