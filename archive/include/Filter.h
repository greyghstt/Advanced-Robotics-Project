#pragma once

// Archived Kalman helper for the old MPU6050 pipeline.
// The active V1 firmware reads BNO055 fused Euler/gyro vectors directly, so this
// helper is kept only as reference and is intentionally excluded from
// compilation.
#if 0

#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

void kalman_set_angle(float initial_pitch, float initial_roll);
float kalman_get_pitch(float accPitch, float gyroY, float dt);
float kalman_get_roll(float accRoll, float gyroX, float dt);

#endif

#include "Kalman.h"

Kalman kalmanPitch;
Kalman kalmanRoll;

void kalman_set_angle(float initial_pitch, float initial_roll) {
    kalmanPitch.setAngle(initial_pitch);
    kalmanRoll.setAngle(initial_roll);
}

float kalman_get_pitch(float accPitch, float gyroY, float dt) {
    return kalmanPitch.getAngle(accPitch, gyroY, dt);
}

float kalman_get_roll(float accRoll, float gyroX, float dt) {
    return kalmanRoll.getAngle(accRoll, gyroX, dt);
}

#endif // Archived Kalman helper
