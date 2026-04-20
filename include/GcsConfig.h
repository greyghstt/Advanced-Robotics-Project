#pragma once

#define ENABLE_UDP_GCS 1

// Replace these placeholders before uploading the firmware.
#define GCS_WIFI_SSID "INSERT_WIFI_SSID_HERE"
#define GCS_WIFI_PASSWORD "INSERT_WIFI_PASSWORD_HERE"
#define GCS_MDNS_HOSTNAME "advanced-robotics"

#define UDP_GCS_LISTEN_PORT 4210
#define UDP_GCS_SEND_PERIOD_MS 50
#define UDP_GCS_TASK_PERIOD_MS 10
#define UDP_GCS_WIFI_TIMEOUT_MS 10000
#define UDP_GCS_ALLOW_ARMED_PID_UPDATE 0
