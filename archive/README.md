# Archive

This folder keeps disabled reference code from earlier telemetry, GCS, sensor,
and control experiments. None of these files are compiled or used by the active
V1 firmware baseline.

## Contents

| Path | Purpose |
| --- | --- |
| `include/BluetoothTelemetry.h` | Archived Bluetooth Serial telemetry/GCS firmware code |
| `include/Telemetry.h` | Archived WiFi HTTP telemetry server code |
| `include/LegacyControl.h` | Archived legacy PID and mixer experiment |
| `include/Filter.h` | Archived Kalman helper for the old MPU6050 path |
| `include/Kalman.h`, `include/Kalman.cpp` | Kalman reference used by the archived IMU path |
| `gcs/gcs_bluetooth.py` | Archived Bluetooth Serial dashboard |

## Restore Note

To reuse any archived feature, move the needed files back into the active
project folders, enable the related mode in `include/GcsConfig.h`, then rebuild
and upload the firmware. UDP GCS remains the only active telemetry mode in the
current baseline.
