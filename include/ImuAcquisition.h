#pragma once

#ifndef IMU_ACQUISITION_H
#define IMU_ACQUISITION_H
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <CopterControl.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // BNO055 I2C address.
float pitch, roll, yaw, delta_yaw, prev_yaw;
float gx, gy, gz;
float dt;
unsigned long previousTime, currentTime;
bool kalman_initialized = false;

// Reference only, not active:
// Older MPU6050 + Kalman variables. The current V1 baseline uses BNO055 sensor
// fusion directly, but this block is kept as a reference if the IMU pipeline is
// rebuilt later.
// float accPitch, accRoll;
// float AcX = 0.0, AcY = 0.0, AcZ = 0.0;
// const int MPU = 0x68;
// Kalman kalmanX;
// Kalman kalmanY;
// Kalman kalmanZ;

void readImuData() {
    // Uncomment during sensor setup if BNO055 calibration needs to be checked.
    // uint8_t sys, gyro_cal, accel, mag;
    // bno.getCalibration(&sys, &gyro_cal, &accel, &mag);
    // Serial.print("CALIB: ");
    // Serial.print(sys); Serial.print(" ,");
    // Serial.print(gyro_cal); Serial.print(" ,");
    // Serial.print(accel); Serial.print(" ,");
    // Serial.println(mag);

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    // BNO055 VECTOR_EULER returns x=heading/yaw, y=roll, z=pitch.
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
    // The current V1 yaw loop still wraps yaw_setpoint in CopterControl.h, but wrapping at
    // the source would avoid a possible yaw jump when heading crosses 0/360 degrees.
    //
    // Proposed change:
    // delta_yaw = yaw - prev_yaw;
    // if (delta_yaw > 180.0f) delta_yaw -= 360.0f;
    // if (delta_yaw < -180.0f) delta_yaw += 360.0f;
    // prev_yaw = yaw;
    delta_yaw = yaw - prev_yaw;
    prev_yaw = yaw;

    // Reference only, not active:
    // Previous raw MPU6050 acquisition and Kalman fusion path. This is disabled
    // because the current hardware path uses BNO055 Euler and gyroscope vectors.
    // currentTime = millis();
    // dt = (currentTime - previousTime) / 1000.0;
    // previousTime = currentTime;
    //
    // // Read accelerometer.
    // Wire.beginTransmission(MPU);
    // Wire.write(0x3B);
    // Wire.endTransmission(false);
    // Wire.requestFrom(MPU, 6);
    //
    // int16_t rawAx = (Wire.read() << 8) | Wire.read();
    // int16_t rawAy = (Wire.read() << 8) | Wire.read();
    // int16_t rawAz = (Wire.read() << 8) | Wire.read();
    //
    // float Ax = rawAx / 16384.0;
    // float Ay = rawAy / 16384.0;
    // float Az = rawAz / 16384.0;
    //
    // // Read gyroscope.
    // Wire.beginTransmission(MPU);
    // Wire.write(0x43);
    // Wire.endTransmission(false);
    // Wire.requestFrom(MPU, 6);
    //
    // int16_t rawGx = (Wire.read() << 8) | Wire.read();
    // int16_t rawGy = (Wire.read() << 8) | Wire.read();
    // int16_t rawGz = (Wire.read() << 8) | Wire.read();
    //
    // gx = rawGx / 131.0;
    // gy = rawGy / 131.0;
    // gz = rawGz / 131.0;
    //
    // accRoll = atan2(Ay, sqrt(Ax * Ax + Az * Az)) * 180.0 / PI;
    // accPitch = atan2(Ax, sqrt(Ay * Ay + Az * Az)) * 180.0 / PI;
    //
    // if (!kalman_initialized) {
    //     kalmanX.setAngle(accRoll);
    //     kalmanY.setAngle(accPitch);
    //     kalman_initialized = true;
    // }
    //
    // roll = kalmanX.getAngle(accRoll, gx, dt);
    // pitch = kalmanY.getAngle(accPitch, gy, dt);
    // yaw += gz * dt;
    //
    // if (yaw >= 360.0) yaw -= 360.0;
    // if (yaw < 0.0) yaw += 360.0;
}

#endif // IMU_ACQUISITION_H




