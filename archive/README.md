# Archive

This folder keeps disabled reference code from earlier telemetry, GCS, and
control experiments for the `version-2-old` branch. None of these files are
compiled or used by the active V2-old firmware build.

## Contents

| Path | Purpose |
| --- | --- |
| `include/BluetoothTelemetry.h` | Archived Bluetooth Serial telemetry/GCS firmware code |
| `include/Telemetry.h` | Archived WiFi HTTP telemetry server code |
| `include/kendali.h` | Archived older control experiment |
| `gcs/gcs.py` | Archived Bluetooth Serial dashboard |

## Branch Note

Unlike the `master` branch, `version-2-old` still keeps `Kalman.h`,
`Kalman.cpp`, and `filter.h` active in `include/` because the older IMU path
still references them.

## Restore Note

To reuse any archived feature, move the needed files back into the active
project folders, enable the related mode in `include/Gcs_config.h`, then
rebuild and upload the firmware. UDP GCS remains the only active telemetry mode
in the current V2-old baseline.
