# Include

This folder contains the main ESP32 firmware headers for the Advanced Robotics
Project. Most project logic is still header-based so it can be called directly
from `src/main.cpp`.

## Important Files

| File | Purpose |
| --- | --- |
| `Actuator.h` | ESC PWM output through LEDC and motor pin mapping |
| `Radio.h` | SBUS receiver reading and remote channel mapping |
| `CopterControl.h` | Diagonal/X mixer, PID gains, trim, and attitude control |
| `Transition.h` | Safety gate, arm/disarm, failsafe, and control loop |
| `ImuAcquisition.h` | BNO055 sensor reading |
| `Ultrasonic.h` | Ultrasonic sensor reading |
| `GcsConfig.h` | Active UDP GCS and WiFi configuration |
| `UdpTelemetry.h` | Active telemetry and PID command handling through UDP |
| `CopterConfig.h` | Angle and throttle limit configuration |
| `ControlModes.h` | Control mode definitions |

## Common Configuration

Active telemetry mode:

```cpp
// GcsConfig.h
#define ENABLE_UDP_GCS 1
```

The previous WiFi HTTP Telemetry, Bluetooth GCS, legacy PID, and Kalman helper
implementations are stored under `../archive/` and are not used by the current
firmware. UDP GCS is the only active telemetry mode.

WiFi credentials:

```cpp
// GcsConfig.h
#define GCS_WIFI_SSID "INSERT_WIFI_SSID_HERE"
#define GCS_WIFI_PASSWORD "INSERT_WIFI_PASSWORD_HERE"
```

Default PID values:

```cpp
// CopterControl.h
gains gain;
```

Motor pins:

```cpp
// Actuator.h
#define MOTOR_1_PIN 33
#define MOTOR_2_PIN 25
#define MOTOR_3_PIN 26
#define MOTOR_4_PIN 27
```

SBUS receiver pin:

```cpp
// Radio.h
SerialSBUS.begin(100000, SERIAL_8E2, 35, 17);
```

## Notes

- Change compile-time configuration only when the firmware will be rebuilt and
  uploaded again.
- Avoid changing radio mapping if the receiver is already reading reliably.
- For early tuning, remove the propellers and disarm before sending new PID
  values.
