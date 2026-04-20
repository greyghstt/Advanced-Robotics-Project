# Advanced Robotics Project

ESP32-based quadcopter flight controller with UDP telemetry and a Python GCS
dashboard. The `master` branch is the current V1 flight baseline that has flown
on hardware.

## Current Status

- Board: ESP32 DOIT DevKit V1.
- Framework: Arduino through PlatformIO.
- Receiver: SBUS on GPIO 35.
- ESC motor output: 50 Hz LEDC PWM.
- Active telemetry mode: UDP GCS only.
- Active branch purpose: stable V1 flight baseline.
- The GCS dashboard is available in the `gcs` folder.
- Default mDNS hostname: `robjut.local`.

## Folder Structure

```text
Advanced-Robotics-Project/
|-- include/              Firmware headers and main configuration
|-- src/                  ESP32 firmware entry point
|-- gcs/                  Python dashboard application
|-- archive/              Disabled legacy telemetry and control references
|-- lib/                  Local libraries or vendored dependencies
|-- test/                 PlatformIO test area
|-- platformio.ini        PlatformIO build/upload configuration
`-- README.md             Main project documentation
```

## Main Hardware

| Function | Pin |
| --- | --- |
| SBUS RX | GPIO 35 |
| Motor 1 | GPIO 33 |
| Motor 2 | GPIO 25 |
| Motor 3 | GPIO 26 |
| Motor 4 | GPIO 27 |

The mixer uses a diagonal/X quadcopter layout:

| Motor | Position | Suggested Rotation |
| --- | --- | --- |
| M1 | Front-left | CW |
| M2 | Front-right | CCW |
| M3 | Rear-right | CW |
| M4 | Rear-left | CCW |

## Setup

1. Install Visual Studio Code.
2. Install the PlatformIO extension.
3. Open the project folder.
4. Connect the ESP32 through USB.
5. Build or upload using PlatformIO.

## Build and Upload

Before uploading, set the WiFi SSID and password in `include/GcsConfig.h`.

Using the PlatformIO UI:

1. Open PlatformIO.
2. Select the `esp32doit-devkit-v1` environment.
3. Click `Build` to compile.
4. Click `Upload` to flash the ESP32.

Using the terminal:

```powershell
cd <project-folder>
platformio run -e esp32doit-devkit-v1
platformio run -e esp32doit-devkit-v1 -t upload
```

Serial Monitor:

```powershell
platformio device monitor -b 115200
```

## Telemetry Mode

Telemetry mode is configured in `include/GcsConfig.h`.

The current firmware uses UDP GCS only:

```cpp
#define ENABLE_UDP_GCS 1
```

Any future telemetry mode change requires rebuilding and uploading the firmware
to the ESP32.

## UDP GCS

UDP GCS is used for the WiFi dashboard with low latency. Connection flow:

1. Turn on the laptop hotspot or connect the laptop and ESP32 to the same WiFi.
2. Upload the firmware with `ENABLE_UDP_GCS 1`.
3. Open Serial Monitor and note the ESP32 IP address.
4. Run the dashboard:

```powershell
cd <project-folder>\gcs
python gcs_udp.py
```

5. Fill the host field with `robjut.local` or the ESP32 IP address.
6. Keep the default remote port at `4210`.
7. Click `Start UDP`.

The dashboard sends `HELLO` to the ESP32. After the ESP32 receives it, telemetry
data is sent back to the dashboard.

## Archived Bluetooth GCS

Bluetooth GCS is kept as archived System A code. It is not used in the current
firmware because UDP GCS is the active telemetry mode.

The Bluetooth dashboard and firmware helpers are kept under `archive/`. To use
this mode again, restore the archived Bluetooth firmware/dashboard code, enable
the Bluetooth mode in `include/GcsConfig.h`, then rebuild and upload the
firmware.

## Archived WiFi HTTP Telemetry

WiFi HTTP Telemetry is kept as archived System A code. It is not used in the
current firmware because UDP GCS is the active telemetry mode.

To use this mode again, restore the archived HTTP telemetry code from
`archive/include/`, enable the HTTP telemetry mode in `include/GcsConfig.h`,
then rebuild and upload the firmware. The previous HTTP telemetry endpoint used
port `8080`.

## Archived Control References

Some older control and sensor experiments are kept for reference, but they are
fully disabled and not compiled in the V1 baseline:

| File | Archived Purpose |
| --- | --- |
| `archive/include/LegacyControl.h` | Older PID and mixer experiment |
| `archive/include/Filter.h` | Older Kalman helper for an MPU6050-style IMU path |
| `archive/include/Telemetry.h` | WiFi HTTP telemetry server |
| `archive/include/BluetoothTelemetry.h` | Bluetooth Serial telemetry/GCS |
| `archive/gcs/gcs_bluetooth.py` | Bluetooth dashboard archive |

Active flight behavior is in `CopterControl.h`, `Transition.h`, `Radio.h`,
`Actuator.h`, `ImuAcquisition.h`, and `UdpTelemetry.h`.

## PID Tuning

Default PID values are defined in `include/CopterControl.h`. The UDP dashboard
can read the active PID values and send tuning updates through its PID panel.

By default, PID updates are blocked while the drone is armed. Disarm the drone
before sending new PID values from the dashboard.

Current V1 default PID baseline:

| Axis | P | I | D / Rate |
| --- | ---: | ---: | ---: |
| Roll | 3.5 | 0.3 | 1.3 |
| Pitch | 3.5 | 0.3 | 1.3 |
| Yaw | 4.0 | 0.0 | 0.5 |

## Safety Notes

- Remove all propellers before upload, motor testing, and early tuning.
- Make sure the motor position and rotation match the hardware table.
- Make sure the ESCs are calibrated and share ground with the ESP32.
- Do not change telemetry mode while the motors are armed.
- Replace the WiFi SSID/password placeholders in `include/GcsConfig.h` before
  uploading the firmware.

## Further Documentation

- `include/README.md` explains the firmware headers.
- `gcs/README.md` explains the Python dashboard.
- `archive/README.md` explains disabled reference code.
- `lib/README.md` explains the local library folder.
- `test/README.md` explains the PlatformIO test folder.
