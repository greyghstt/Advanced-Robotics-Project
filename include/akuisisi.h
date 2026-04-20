#pragma once

#ifndef AKUISISI_H
#define AKUISISI_H
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <Kalman.h>
#include <Copter_control.h>
#include <Adafruit_BNO055.h>
#include "filter.h"

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // Alamat I2C BNO055
float pitch, roll, yaw, delta_yaw, prev_yaw;
float gx, gy, gz;
// float accPitch, accRoll; krn pake bno
// float AcX = 0.0, AcY = 0.0, AcZ = 0.0;
// const int MPU = 0x68;  krn pake bno // Alamat I2C MPU6050
float dt;
unsigned long previousTime, currentTime;
bool kalman_initialized = false;

// Kalman kalmanX;
// Kalman kalmanY;
// Kalman kalmanZ;

void ambil_data_imu() {
    // uint8_t sys, gyro_cal, accel, mag;
    // bno.getCalibration(&sys, &gyro_cal, &accel, &mag);

    // Serial.print("CALIB: ");
    // Serial.print(sys); Serial.print(" ,");
    // Serial.print(gyro_cal); Serial.print(" ,");
    // Serial.print(accel); Serial.print(" ,");
    // Serial.println(mag); //sementara utk cek kalibrasi

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    // BNO055 VECTOR_EULER dibaca dari register H/R/P:
    // x = heading/yaw, y = roll, z = pitch.
    yaw = euler.x();
    roll = euler.y();
    pitch = euler.z();

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    gx = gyro.x();
    gy = gyro.y();
    gz = gyro.z();

    if(yaw >= 360.0) yaw -= 360.0;
    if(yaw < 0.0) yaw += 360.0;

    // TODO(yaw stability, untested): wrap delta_yaw here before it is used by yaw control.
    // The current V1 yaw loop still wraps yaw_setpoint in Copter_control.h, but wrapping at
    // the source would avoid a possible yaw jump when heading crosses 0/360 degrees.
    //
    // Proposed change:
    // delta_yaw = yaw - prev_yaw;
    // if (delta_yaw > 180.0f) delta_yaw -= 360.0f;
    // if (delta_yaw < -180.0f) delta_yaw += 360.0f;
    // prev_yaw = yaw;
    delta_yaw = yaw - prev_yaw;
    prev_yaw = yaw;
    // currentTime = millis();
    // dt = (currentTime - previousTime) / 1000.0;
    // previousTime = currentTime;

    // // Baca data akselerometer
    // // Wire.beginTransmission(MPU);
    // // Wire.write(0x3B);
    // // Wire.endTransmission(false);
    // // Wire.requestFrom(MPU, 6); //krn pake bno

    // int16_t rawAx = (Wire.read() << 8) | Wire.read();
    // int16_t rawAy = (Wire.read() << 8) | Wire.read();
    // int16_t rawAz = (Wire.read() << 8) | Wire.read();

    // float Ax = rawAx / 16384.0;
    // float Ay = rawAy / 16384.0;
    // float Az = rawAz / 16384.0;

    // // Baca data gyroscope
    // // Wire.beginTransmission(MPU);
    // // Wire.write(0x43);
    // // Wire.endTransmission(false);
    // // Wire.requestFrom(MPU, 6); //krn pake bno

    // int16_t rawGx = (Wire.read() << 8) | Wire.read();
    // int16_t rawGy = (Wire.read() << 8) | Wire.read();
    // int16_t rawGz = (Wire.read() << 8) | Wire.read();

    // gx = rawGx / 131.0;
    // gy = rawGy / 131.0;
    // gz = rawGz / 131.0;

    // accRoll  = atan2(Ay, sqrt(Ax * Ax + Az * Az)) * 180.0 / PI;
    // accPitch = atan2(Ax, sqrt(Ay * Ay + Az * Az)) * 180.0 / PI;

    // // pitch = 0.98 * (pitch + (gy) * dt) + (0.02) * accPitch;
    // // roll = 0.98 * (roll + (gx) * dt) + (0.02) * accRoll;

    // // Inisialisasi pertama kalman
    // if (!kalman_initialized) {
    //     kalman_set_angle(accPitch, accRoll);
    //     kalman_initialized = true;
    // }
    // pitch = kalman_get_pitch(accPitch, gy, dt);
    // roll  = kalman_get_roll(accRoll, gx, dt); krn pake bno

    // yaw += gz * dt;
    // delta_yaw = yaw - prev_yaw;
    // prev_yaw = yaw;

    // if (yaw >= 360.0) yaw -= 360.0;
    // if (yaw < 0.0) yaw += 360.0;

    // // Gunakan Kalman Filter
    // Roll = kalmanX.getAngle(accRoll, gx, dt);
    // Pitch = kalmanY.getAngle(accPitch, gy, dt);

    // // Gunakan Kalman Filter untuk Yaw (opsional)
    // Yaw = kalmanZ.getAngle(Yaw, gz, dt);
}

#endif // AKUISISI_H




