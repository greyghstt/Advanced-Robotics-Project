# Advanced Robotics Project GCS

Python dashboard for reading ESP32 telemetry and sending PID tuning updates.
The active dashboard is the WiFi UDP version.

## Files

| File | Purpose |
| --- | --- |
| `gcs_udp.py` | Active WiFi UDP dashboard |
| `requirements.txt` | Python dashboard dependencies |

## Dependencies

Use the Python installation that already has PySide6 and pyserial available.

```powershell
cd <project-folder>\gcs
python -m pip install -r requirements.txt
```

## UDP GCS

UDP is the active firmware mode. Make sure `include/Gcs_config.h` keeps UDP
enabled:

```cpp
#define ENABLE_UDP_GCS 1
```

Run the dashboard:

```powershell
cd <project-folder>\gcs
python gcs_udp.py
```

Connection flow:

1. Turn on the laptop hotspot or connect the laptop and ESP32 to the same WiFi.
2. Upload the firmware to the ESP32.
3. Open Serial Monitor and note the ESP32 IP address.
4. Run `python gcs_udp.py`.
5. Fill the host field with `advanced-robotics.local` or the ESP32 IP address.
6. Keep the default remote port at `4210`.
7. Keep the default local port at `4211`.
8. Click `Start UDP`.

The dashboard sends `HELLO` to the ESP32. After the ESP32 receives `HELLO`,
telemetry starts streaming to the dashboard.

## Dashboard Features

- Clear UDP connection state, telemetry age, and received packet count.
- Telemetry timeout warning when packets stop arriving.
- PID send buttons are locked while `arm=1`.
- Firmware command responses are shown in the header and log panel.
- PID values can be requested automatically and loaded from telemetry.
- PID presets can be loaded, saved, and deleted locally.
- Telemetry can be recorded to CSV files under `gcs/logs/`.
- Motor PWM values are shown as live bars.
- Roll and pitch are shown on the attitude chart with a `+/-30 deg` limit.

## Archived Bluetooth GCS

Bluetooth GCS is kept as archived System A code. It is not used in the current
firmware because UDP GCS is the active mode.

The Bluetooth dashboard has been moved to `../archive/gcs/gcs.py`. To use this
mode again, restore the archived Bluetooth firmware/dashboard code, enable the
Bluetooth mode in `include/Gcs_config.h`, then rebuild and upload the firmware.

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
