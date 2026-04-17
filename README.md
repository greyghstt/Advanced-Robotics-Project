# Advanced Robotics Project

ESP32 firmware for a quadcopter developed for the Advanced Robotics course.

## Current Status

- Board: ESP32 DOIT DevKit V1.
- Framework: Arduino through PlatformIO.
- Receiver: SBUS on GPIO 35.
- ESC motor output: 50 Hz LEDC PWM.
- Active telemetry mode: UDP GCS only.
- WiFi HTTP Telemetry and Bluetooth GCS are kept as archived code, but they
  are not used in the current firmware.
- The GCS dashboard is available in the `gcs` folder.
- Default mDNS hostname: `robjut.local`.

## Folder Structure

```text
Advanced-Robotics-Project/
|-- include/              Firmware headers and main configuration
|-- src/                  ESP32 firmware entry point
|-- gcs/                  Python dashboard application
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

Current local project path:

```text
C:\vscode\Advanced-Robotics-Project
```

4. Connect the ESP32 through USB.
5. Build or upload using PlatformIO.

## Build and Upload

Using the PlatformIO UI:

1. Open PlatformIO.
2. Select the `esp32doit-devkit-v1` environment.
3. Click `Build` to compile.
4. Click `Upload` to flash the ESP32.

Using the terminal:

```powershell
cd C:\vscode\Advanced-Robotics-Project
platformio run -e esp32doit-devkit-v1
platformio run -e esp32doit-devkit-v1 -t upload
```

Serial Monitor:

```powershell
platformio device monitor -b 115200
```

## Telemetry Mode

Telemetry mode is configured in `include/Gcs_config.h`.

The current firmware uses UDP GCS only:

```cpp
// System A is kept as archived code and is not used in this version.
// #define ENABLE_WIFI_HTTP_TELEMETRY 0
// #define ENABLE_BT_GCS 0
#define ENABLE_UDP_GCS 1
```

The WiFi HTTP Telemetry and Bluetooth GCS blocks are kept as comments/archive
in the firmware. They are not compiled in the current version. Any future mode
change still requires rebuilding and uploading the firmware to the ESP32.

## UDP GCS

UDP GCS is used for the WiFi dashboard with low latency. Connection flow:

1. Turn on the laptop hotspot or connect the laptop and ESP32 to the same WiFi.
2. Upload the firmware with `ENABLE_UDP_GCS 1`.
3. Open Serial Monitor and note the ESP32 IP address.
4. Run the dashboard:

```powershell
cd C:\vscode\Advanced-Robotics-Project\gcs
python gcs_udp.py
```

5. Fill the host field with `robjut.local` or the ESP32 IP address.
6. Keep the default remote port at `4210`.
7. Click `Start UDP`.

The dashboard sends `HELLO` to the ESP32. After the ESP32 receives it, telemetry
data is sent back to the dashboard.

<!--
## Bluetooth GCS

This section is kept as System A archive. This mode is not used in the current
firmware because UDP GCS is the active telemetry mode.

Bluetooth GCS can be used again if the Bluetooth mode is re-enabled:

```cpp
#define ENABLE_WIFI_HTTP_TELEMETRY 0
#define ENABLE_BT_GCS 1
#define ENABLE_UDP_GCS 0
```

Run the dashboard:

```powershell
cd C:\vscode\Advanced-Robotics-Project\gcs
python gcs.py
```

Pair Windows with the Bluetooth GCS device, select the COM port, then click
`Connect`.

## WiFi HTTP Telemetry

This section is kept as System A archive. This mode is not used in the current
firmware because UDP GCS is the active telemetry mode.

HTTP telemetry can be used again if the mode is re-enabled:

```cpp
#define ENABLE_WIFI_HTTP_TELEMETRY 1
#define ENABLE_BT_GCS 0
#define ENABLE_UDP_GCS 0
```

After the ESP32 connects to WiFi, open:

```text
http://robjut.local:8080
```

If mDNS does not resolve, use the ESP32 IP address from Serial Monitor.
-->

## PID Tuning

Default PID values are defined in `include/Copter_control.h`. The UDP dashboard
can read the active PID values and send tuning updates through its PID panel.

By default, PID updates are blocked while the drone is armed. Disarm the drone
before sending new PID values from the dashboard.

## Safety Notes

- Remove all propellers before upload, motor testing, and early tuning.
- Make sure the motor position and rotation match the hardware table.
- Make sure the ESCs are calibrated and share ground with the ESP32.
- Do not change telemetry mode while the motors are armed.
- Keep WiFi SSID/password values only in a private repository, or replace them
  before publishing the project.

## Further Documentation

- `include/README.md` explains the firmware headers.
- `gcs/README.md` explains the Python dashboard.
- `lib/README.md` explains the local library folder.
- `test/README.md` explains the PlatformIO test folder.
