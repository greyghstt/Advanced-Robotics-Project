#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <Arduino.h> // Ensure this library is included for Arduino macros like OUTPUT
#ifndef OUTPUT
#define OUTPUT 0x1 // Define OUTPUT if not already defined
#endif
#include <ESP32Servo.h>
#include "Copter_control.h"

// Define the constrain function if not already defined
#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

// Pin konfigurasi motor
#define MOTOR_1_PIN  33  
#define MOTOR_2_PIN  25
#define MOTOR_3_PIN  26
#define MOTOR_4_PIN  27

// Variabel global
int control1, control2, control3, control4;
Servo motor1, motor2, motor3, motor4;
float m1_pwm, m2_pwm, m3_pwm, m4_pwm;
uint16_t servo_max = 2012;
uint16_t servo_min = 988;
uint16_t throttle;

// Fungsi untuk menginisialisasi aktuator
void init_actuator() {
    pinMode(MOTOR_1_PIN, OUTPUT);
    pinMode(MOTOR_2_PIN, OUTPUT);
    pinMode(MOTOR_3_PIN, OUTPUT);
    pinMode(MOTOR_4_PIN, OUTPUT);

    motor1.attach(MOTOR_1_PIN);
    motor2.attach(MOTOR_2_PIN);
    motor3.attach(MOTOR_3_PIN);
    motor4.attach(MOTOR_4_PIN);

    // Set motor ke nilai minimum (988 µs)
    motor1.writeMicroseconds(servo_min);
    motor2.writeMicroseconds(servo_min);
    motor3.writeMicroseconds(servo_min);
    motor4.writeMicroseconds(servo_min);

    Serial.println("Actuator setup complete");
}

// Fungsi untuk menghitung output motor berdasarkan throttle dan kontrol
void copter_calcOutput(int16_t ch_thr) {
    throttle = ch_thr;

    // Hitung kontrol motor berdasarkan omega2 dan konstanta M_CONST
    control1 = (int)(omega2[0] * M_CONST);
    control2 = (int)(omega2[1] * M_CONST);
    control3 = (int)(omega2[2] * M_CONST);
    control4 = (int)(omega2[3] * M_CONST);

    // Batasi nilai kontrol dalam rentang -400 hingga 400
    control1 = constrain(control1, -400, 400);
    control2 = constrain(control2, -400, 400);
    control3 = constrain(control3, -400, 400);
    control4 = constrain(control4, -400, 400);

    // Jika arming aktif, hitung nilai PWM motor
    if (arming) {
        m1_pwm = control1 + throttle;
        m2_pwm = control2 + throttle;
        m3_pwm = control3 + throttle;
        m4_pwm = control4 + throttle;
    } else {
        // Jika arming tidak aktif, set motor ke nilai minimum
        m1_pwm = 1000;
        m2_pwm = 1000;
        m3_pwm = 1000;
        m4_pwm = 1000;
    }

    // Batasi nilai PWM dalam rentang servo_min hingga servo_max
    m1_pwm = constrain(m1_pwm, servo_min, servo_max);
    m2_pwm = constrain(m2_pwm, servo_min, servo_max);
    m3_pwm = constrain(m3_pwm, servo_min, servo_max);
    m4_pwm = constrain(m4_pwm, servo_min, servo_max);
}

// Fungsi untuk mengirim nilai PWM ke motor
void motor_loop(int pwm1, int pwm2, int pwm3, int pwm4) {
    motor1.writeMicroseconds(pwm1);
    motor2.writeMicroseconds(pwm2);
    motor3.writeMicroseconds(pwm3);
    motor4.writeMicroseconds(pwm4);
}

// Fungsi untuk mengonversi nilai persen throttle ke PWM
float PercenttoPWM(float percent) {
    return map(percent, 0, 100, servo_min, servo_max);
}

#endif // ACTUATOR_H