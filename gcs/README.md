# Advanced Robotics Project GCS

Python dashboard for reading ESP32 telemetry and sending PID tuning updates.
The active dashboard is the WiFi UDP version.

## Files

| File | Purpose |
| --- | --- |
| `gcs_udp.py` | Active WiFi UDP dashboard |
| `gcs.py` | Archived Bluetooth Serial dashboard |
| `requirements.txt` | Python dashboard dependencies |

## Dependencies

Use the Python installation that already has PySide6 and pyserial available.

```powershell
cd C:\vscode\Project-Robjut\gcs
python -m pip install -r requirements.txt
```

## UDP GCS

UDP is the active firmware mode. Make sure `include/Gcs_config.h` keeps UDP
enabled:

```cpp
// System A is kept as archived code and is not used in this version.
// #define ENABLE_WIFI_HTTP_TELEMETRY 0
// #define ENABLE_BT_GCS 0
#define ENABLE_UDP_GCS 1
```

Run the dashboard:

```powershell
cd C:\vscode\Project-Robjut\gcs
python gcs_udp.py
```

Connection flow:

1. Turn on the laptop hotspot or connect the laptop and ESP32 to the same WiFi.
2. Upload the firmware to the ESP32.
3. Open Serial Monitor and note the ESP32 IP address.
4. Run `python gcs_udp.py`.
5. Fill the host field with `robjut.local` or the ESP32 IP address.
6. Keep the default remote port at `4210`.
7. Keep the default local port at `4211`.
8. Click `Start UDP`.

The dashboard sends `HELLO` to the ESP32. After the ESP32 receives `HELLO`,
telemetry starts streaming to the dashboard.

<!--
## Bluetooth GCS

This section is kept as System A archive. Bluetooth GCS is not used in the
current firmware because UDP GCS is the active mode.

To use this mode again, the archived Bluetooth firmware code must be restored
and the mode must be enabled in `include/Gcs_config.h`.
-->

## Telemetry Format

The firmware can send telemetry in CSV and JSON format. The dashboard uses:

- radio, failsafe, arm, and mode status
- remote PWM inputs
- motor PWM outputs
- roll, pitch, and yaw
- gyro values
- active PID values
- trim values

## Firmware Commands

The UDP dashboard normally sends commands automatically. The supported protocol
commands are:

```text
HELLO
PING
HEADER
GET CSV
GET JSON
PID
SET <NAME> <VALUE>
```

Use the dashboard PID fields for tuning instead of manually typing raw command
values. By default, the firmware rejects PID updates while `arm=1`; disarm the
drone before sending PID updates.

## Mode Note

Telemetry mode uses compile-time `#define` values, so changing modes requires
building and uploading the firmware again.
