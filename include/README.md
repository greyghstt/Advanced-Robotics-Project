# Include

This folder contains the active ESP32 firmware headers for Version 4 of the
Advanced Robotics Project flight controller.

Version 4 is intentionally cleaner than the older branches: archived Bluetooth
GCS, HTTP telemetry, legacy Kalman helpers, and experimental control files were
removed from this branch. UDP GCS is the only active telemetry path.

## Active Files

| File | Purpose |
| --- | --- |
| `Actuator.h` | ESC PWM output through ESP32 LEDC and motor pin mapping |
| `Radio.h` | SBUS receiver reading and channel mapping |
| `ImuSensor.h` | BNO055 setup, attitude, gyro, calibration, and freshness state |
| `FlightControl.h` | V4 cascaded attitude/rate controller and Quad-X mixer |
| `FlightSafety.h` | Arming gate, failsafe, pre-arm checks, and control loop dispatch |
| `FlightConfig.h` | Shared throttle and flight limits |
| `ControlModes.h` | Mode placeholders and altitude-hold helper |
| `GcsConfig.h` | UDP GCS and WiFi configuration |
| `UdpTelemetry.h` | Active telemetry and PID command handling through UDP |
| `Ultrasonic.h` | Ultrasonic sensor helper kept for future altitude work |

## V4 Control Summary

The controller follows a simple multicopter pattern:

```text
RC stick -> target angle/rate -> PID/rate correction -> Quad-X motor mix
```

The active loop includes:

- RC deadband near stick center
- roll/pitch angle-to-rate control
- gyro-rate damping
- optional integral with clamp
- high-throttle P/D attenuation
- motor correction scaling near PWM limits
- pre-arm checks before motors are allowed to start

## Active Telemetry Mode

```cpp
// GcsConfig.h
#define ENABLE_UDP_GCS 1
```

## Motor Pins

```cpp
// Actuator.h
#define MOTOR_1_PIN 33
#define MOTOR_2_PIN 25
#define MOTOR_3_PIN 26
#define MOTOR_4_PIN 27
```

## SBUS Receiver Pin

```cpp
// Radio.h
SerialSBUS.begin(100000, SERIAL_8E2, 35, 17);
```

## Safety Notes

- Remove all propellers before upload, motor testing, and early V4 tuning.
- Validate correction direction before increasing throttle.
- V4 starts with conservative defaults; tuning is expected.
