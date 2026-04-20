# Include

This folder contains the main ESP32 firmware headers for the Advanced Robotics
Project. Most project logic is still header-based so it can be called directly
from `src/main.cpp`.

## Important Files

| File | Purpose |
| --- | --- |
| `Actuator.h` | ESC PWM output through LEDC and motor pin mapping |
| `Radio.h` | SBUS receiver reading and remote channel mapping |
| `Copter_control.h` | Diagonal/X mixer, PID gains, trim, and attitude control |
| `Transisition.h` | Safety gate, arm/disarm, failsafe, and control loop |
| `akuisisi.h` | BNO055 sensor reading |
| `Ultrasonik.h` | Ultrasonic sensor reading |
| `Gcs_config.h` | Active UDP GCS and WiFi configuration |
| `UdpTelemetry.h` | Active telemetry and PID command handling through UDP |
| `Kalman.h`, `Kalman.cpp` | Active Kalman helper used by the V2-old IMU path |
| `Copter_config.h` | Angle and throttle limit configuration |
| `Control_modes.h` | Control mode definitions |
| `filter.h` | Active filter helper used by the V2-old IMU path |

## Common Configuration

Active telemetry mode:

```cpp
// Gcs_config.h
#define ENABLE_UDP_GCS 1
```

The previous WiFi HTTP Telemetry and Bluetooth GCS implementations are archived
and not used by the current firmware. UDP GCS is the only active telemetry
mode.

Archived files are stored under `../archive/`:

| Archived File | Purpose |
| --- | --- |
| `../archive/include/BluetoothTelemetry.h` | Bluetooth Serial telemetry/GCS |
| `../archive/include/Telemetry.h` | WiFi HTTP telemetry server |
| `../archive/include/kendali.h` | Older control experiment |
| `../archive/gcs/gcs.py` | Bluetooth dashboard archive |

Unlike the `master` branch, the V2-old firmware still keeps `Kalman.h`,
`Kalman.cpp`, and `filter.h` active because the legacy IMU path depends on
them.

WiFi credentials:

```cpp
// Gcs_config.h
#define GCS_WIFI_SSID "INSERT_WIFI_SSID_HERE"
#define GCS_WIFI_PASSWORD "INSERT_WIFI_PASSWORD_HERE"
```

Default PID values:

```cpp
// Copter_control.h
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
- Before uploading, replace the WiFi placeholders in `Gcs_config.h`.
- For early tuning, remove the propellers and disarm before sending new PID
  values.
