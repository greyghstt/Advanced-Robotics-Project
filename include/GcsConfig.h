#pragma once

// Select the active telemetry/GCS mode here.
// This is compile-time configuration, so every mode change requires a rebuild
// and upload to the ESP32.
// System A modes are archived and disabled in this V1 baseline.
// #define ENABLE_WIFI_HTTP_TELEMETRY 0
// #define ENABLE_BT_GCS 0
#define ENABLE_UDP_GCS 1

// WiFi settings used by UDP GCS.
// Replace these placeholders with the laptop hotspot or router credentials
// before uploading the firmware.
#define GCS_WIFI_SSID "INSERT_WIFI_SSID_HERE"
#define GCS_WIFI_PASSWORD "INSERT_WIFI_PASSWORD_HERE"
#define GCS_MDNS_HOSTNAME "advanced-robotics"

// UDP GCS runtime settings.
#define UDP_GCS_LISTEN_PORT 4210
#define UDP_GCS_SEND_PERIOD_MS 50
#define UDP_GCS_TASK_PERIOD_MS 10
#define UDP_GCS_WIFI_TIMEOUT_MS 10000
#define UDP_GCS_ALLOW_ARMED_PID_UPDATE 0
