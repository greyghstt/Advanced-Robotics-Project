#pragma once

// Pilih mode GCS/telemetry di sini.
// Karena ini compile-time config, setiap ganti mode perlu upload ulang ke ESP32.
#define ENABLE_WIFI_HTTP_TELEMETRY 0
#define ENABLE_BT_GCS 0
#define ENABLE_UDP_GCS 1

// Dipakai oleh WiFi HTTP telemetry dan UDP GCS.
#define GCS_WIFI_SSID "grey"
#define GCS_WIFI_PASSWORD "1234567@"
#define GCS_MDNS_HOSTNAME "robjut"

// UDP GCS.
#define UDP_GCS_LISTEN_PORT 4210
#define UDP_GCS_SEND_PERIOD_MS 50
#define UDP_GCS_TASK_PERIOD_MS 10
#define UDP_GCS_WIFI_TIMEOUT_MS 10000
#define UDP_GCS_ALLOW_ARMED_PID_UPDATE 0
