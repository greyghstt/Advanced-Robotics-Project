#pragma once

#ifndef AKUISISI_H
#define AKUISISI_H
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Kalman.h>
#include "filter.h"


float pitch, roll, yaw, delta_yaw, prev_yaw, gx, gy, gz;
float accPitch, accRoll; 
// float AcX = 0.0, AcY = 0.0, AcZ = 0.0;
const int MPU = 0x68;  // Alamat I2C MPU6050
float dt;
unsigned long previousTime, currentTime;
bool kalman_initialized = false;

// Kalman kalmanX;
// Kalman kalmanY;
// Kalman kalmanZ;

void ambil_data_imu() {
    currentTime = millis();
    dt = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    // Baca data akselerometer
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);

    int16_t rawAx = (Wire.read() << 8) | Wire.read();
    int16_t rawAy = (Wire.read() << 8) | Wire.read();
    int16_t rawAz = (Wire.read() << 8) | Wire.read();

    float Ax = rawAx / 16384.0;
    float Ay = rawAy / 16384.0;
    float Az = rawAz / 16384.0;

    // Baca data gyroscope
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6);

    int16_t rawGx = (Wire.read() << 8) | Wire.read();
    int16_t rawGy = (Wire.read() << 8) | Wire.read();
    int16_t rawGz = (Wire.read() << 8) | Wire.read();

    gx = rawGx / 131.0;
    gy = rawGy / 131.0;
    gz = rawGz / 131.0;

    accRoll  = atan2(Ay, sqrt(Ax * Ax + Az * Az)) * 180.0 / PI;
    accPitch = atan2(Ax, sqrt(Ay * Ay + Az * Az)) * 180.0 / PI;

    // pitch = 0.98 * (pitch + (gy) * dt) + (0.02) * accPitch;
    // roll = 0.98 * (roll + (gx) * dt) + (0.02) * accRoll;

    // Inisialisasi pertama kalman
    if (!kalman_initialized) {
        kalman_set_angle(accPitch, accRoll);
        kalman_initialized = true;
    }   
    pitch = kalman_get_pitch(accPitch, gy, dt);
    roll  = kalman_get_roll(accRoll, gx, dt);

    yaw += gz * dt;
    delta_yaw = yaw - prev_yaw;
    prev_yaw = yaw;
    
    if (yaw >= 360.0) yaw -= 360.0;
    if (yaw < 0.0) yaw += 360.0;
    
    // // Gunakan Kalman Filter
    // Roll = kalmanX.getAngle(accRoll, gx, dt);
    // Pitch = kalmanY.getAngle(accPitch, gy, dt);

    // // Gunakan Kalman Filter untuk Yaw (opsional)
    // Yaw = kalmanZ.getAngle(Yaw, gz, dt);
}

#endif // AKUISISI_H




