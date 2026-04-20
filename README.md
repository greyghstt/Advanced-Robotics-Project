# Advanced Robotics Project

ESP32-based quadcopter flight controller with SBUS receiver input, BNO055 IMU
feedback, ESC PWM output, UDP telemetry, and a Python GCS dashboard.

## Current Status

- Board: ESP32 DOIT DevKit V1.
- Framework: Arduino through PlatformIO.
- Receiver: SBUS on GPIO 35.
- ESC motor output: 50 Hz LEDC PWM.
- Active branch: Version 4 experimental controller.
- Active telemetry mode: UDP GCS.
- Dashboard application: `gcs/gcs_udp.py`.
- Default mDNS hostname: `robjut.local`.

## Folder Structure

```text
Advanced-Robotics-Project/
|-- include/              Firmware headers and main configuration
|-- src/                  ESP32 firmware entry point
|-- gcs/                  Python dashboard application
|-- lib/                  Reserved for local libraries
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

Example local project path for this worktree:

```text
C:\vscode\ARP-V4
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
cd C:\vscode\ARP-V4
platformio run -e esp32doit-devkit-v1
platformio run -e esp32doit-devkit-v1 -t upload
```

Serial Monitor:

```powershell
platformio device monitor -b 115200
```

## Telemetry Mode

Telemetry mode is configured in `include/GcsConfig.h`.

The Version 4 firmware uses UDP GCS only:

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
cd C:\vscode\ARP-V4\gcs
python gcs_udp.py
```

5. Fill the host field with `robjut.local` or the ESP32 IP address.
6. Keep the default remote port at `4210`.
7. Click `Start UDP`.

The dashboard sends `HELLO` to the ESP32. After the ESP32 receives it, telemetry
data is sent back to the dashboard.

## Version 4 Firmware

Version 4 is a clean experimental flight-controller branch. It keeps the same
ESP32 board, motor pins, SBUS receiver pin, UDP dashboard, and Python GCS, while
the flight-control layer is rebuilt around a clearer cascaded controller:

```text
RC stick -> target angle/rate -> PID/rate correction -> Quad-X motor mix
```

The old archived Bluetooth GCS, HTTP telemetry, Kalman helper, and legacy
experimental control files are not part of this branch. UDP GCS is the active
telemetry mode.

## Version 4 Safety State

The current V4 safety gate keeps the checks that are useful during tuning:

- radio frame validity, SBUS failsafe, and radio timeout
- disarm state
- actuator readiness
- recent IMU update
- motor start throttle threshold at `1100 us`

The tilt pre-arm check and high-throttle pre-arm block are disabled in this
worktree because manual bench testing often requires holding the airframe at a
large angle. Roll, pitch, yaw, and mixer output limiting are also temporarily
commented in `include/FlightControl.h` for open-loop tuning tests. Final ESC
PWM output is still constrained to the configured `1000-2000 us` range in
`include/Actuator.h`.

## PID Tuning

Default PID values are defined in `include/FlightControl.h`. The UDP dashboard
can read the active PID values and send tuning updates through its PID panel.

By default, PID updates are blocked while the drone is armed. Disarm the drone
before sending new PID values from the dashboard.

## Safety Notes

- Remove all propellers before upload, motor testing, and early tuning.
- Make sure the motor position and rotation match the hardware table.
- Make sure the ESCs are calibrated and share ground with the ESP32.
- Do not change telemetry mode while the motors are armed.
- Start V4 tuning with small gains because the temporary open-loop tuning state
  allows larger controller corrections before the final ESC PWM clamp.
- Keep WiFi SSID/password values only in a private repository, or replace them
  before publishing the project.

## Further Documentation

- `include/README.md` explains the Version 4 firmware headers.
- `gcs/README.md` explains the Python dashboard.
- `lib/README.md` explains the local library folder.
- `test/README.md` explains the PlatformIO test folder.
