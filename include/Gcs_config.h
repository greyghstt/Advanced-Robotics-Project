#pragma once

// Select the GCS or telemetry mode here.
// This is compile-time configuration, so any mode change requires rebuilding
// and uploading the firmware again.
// System A is kept only as archived reference code in this branch.
// #define ENABLE_WIFI_HTTP_TELEMETRY 0
// #define ENABLE_BT_GCS 0
#define ENABLE_UDP_GCS 1

// Used by the active UDP GCS mode.
#define GCS_WIFI_SSID "INSERT_WIFI_SSID_HERE"
#define GCS_WIFI_PASSWORD "INSERT_WIFI_PASSWORD_HERE"
#define GCS_MDNS_HOSTNAME "robjut"

// UDP GCS settings.
#define UDP_GCS_LISTEN_PORT 4210
#define UDP_GCS_SEND_PERIOD_MS 50
#define UDP_GCS_TASK_PERIOD_MS 10
#define UDP_GCS_WIFI_TIMEOUT_MS 10000
#define UDP_GCS_ALLOW_ARMED_PID_UPDATE 0
