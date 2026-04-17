#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

static const uint8_t IMU_BNO055_ADDRESS = 0x28;
static const uint16_t IMU_STARTUP_DELAY_MS = 1000;
static const float IMU_RAD_TO_DEG = 57.2957795f;

Adafruit_BNO055 bno = Adafruit_BNO055(55, IMU_BNO055_ADDRESS);

float pitch = 0.0f;
float roll = 0.0f;
float yaw = 0.0f;
float delta_yaw = 0.0f;
float prev_yaw = 0.0f;
float gx = 0.0f;
float gy = 0.0f;
float gz = 0.0f;
float dt = 0.0f;

uint8_t imu_cal_sys = 0;
uint8_t imu_cal_gyro = 0;
uint8_t imu_cal_accel = 0;
uint8_t imu_cal_mag = 0;
bool imu_ready = false;
unsigned long imu_last_update_ms = 0;
unsigned long previousTime = 0;
unsigned long currentTime = 0;
uint32_t imu_sample_count = 0;

float imu_wrap_180(float angle_deg) {
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

bool imu_setup() {
    if (!bno.begin()) {
        imu_ready = false;
        return false;
    }

    delay(IMU_STARTUP_DELAY_MS);
    bno.setExtCrystalUse(true);

    previousTime = millis();
    imu_ready = true;
    return true;
}

void imu_update() {
    if (!imu_ready) {
        return;
    }

    currentTime = millis();
    dt = (currentTime - previousTime) / 1000.0f;
    previousTime = currentTime;

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    yaw = euler.x();
    roll = euler.y();
    pitch = euler.z();

    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    // BNO055 gyro vector is rad/s. The V4 rate controller uses deg/s.
    gx = gyro.x() * IMU_RAD_TO_DEG;
    gy = gyro.y() * IMU_RAD_TO_DEG;
    gz = gyro.z() * IMU_RAD_TO_DEG;

    if (yaw >= 360.0f) yaw -= 360.0f;
    if (yaw < 0.0f) yaw += 360.0f;

    delta_yaw = imu_wrap_180(yaw - prev_yaw);
    prev_yaw = yaw;
    imu_last_update_ms = currentTime;
    imu_sample_count++;

    bno.getCalibration(&imu_cal_sys, &imu_cal_gyro, &imu_cal_accel, &imu_cal_mag);
}

bool imu_recent(unsigned long timeout_ms) {
    return imu_ready && ((millis() - imu_last_update_ms) <= timeout_ms);
}
