# Include

This folder contains the active ESP32 firmware headers for the experimental
flight-controller branch of the Advanced Robotics Project.

This branch is intentionally cleaner than the older branches: Bluetooth GCS,
HTTP telemetry, legacy Kalman helpers, and older experimental control files are
not part of this worktree. UDP GCS is the only active telemetry path.

## Active Files

| File | Purpose |
| --- | --- |
| `Actuator.h` | ESC PWM output through ESP32 LEDC and motor pin mapping |
| `Radio.h` | SBUS receiver reading and channel mapping |
| `ImuSensor.h` | BNO055 setup, attitude, gyro, calibration, and freshness state |
| `FlightControl.h` | Cascaded attitude/rate controller and Quad-X mixer |
| `FlightSafety.h` | Arming gate, failsafe, pre-arm checks, and control loop dispatch |
| `FlightConfig.h` | Shared throttle and flight limits |
| `ControlModes.h` | Mode placeholders and altitude-hold helper |
| `GcsConfig.h` | UDP GCS and WiFi configuration |
| `UdpTelemetry.h` | Active telemetry and PID command handling through UDP |
| `Ultrasonic.h` | Ultrasonic sensor helper kept for future altitude work |

## Control Summary

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
- radio, actuator, and IMU freshness checks before motors are allowed to start
- a motor start throttle threshold at `1100 us`

Motor correction limiting and slew limiting remain disabled in
`FlightControl.h` for open-loop tuning tests. The final ESC PWM output remains
constrained in `Actuator.h`.

## Active Telemetry Mode

```cpp
// GcsConfig.h
#define ENABLE_UDP_GCS 1
```

WiFi credentials:

```cpp
// GcsConfig.h
#define GCS_WIFI_SSID "INSERT_WIFI_SSID_HERE"
#define GCS_WIFI_PASSWORD "INSERT_WIFI_PASSWORD_HERE"
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

- Remove all propellers before upload, motor testing, and early tuning.
- Validate correction direction before increasing throttle.
- The tilt pre-arm check is disabled in this branch to support manual tilt
  testing while the airframe is held by hand.
- Replace the WiFi placeholders in `GcsConfig.h` before uploading.
- This branch is experimental; tuning is expected before any free-flight
  attempt.
