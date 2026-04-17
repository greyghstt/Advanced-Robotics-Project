#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>

#include "Gcs_config.h"
#include "Radio.h"
#include "Actuator.h"
#include "akuisisi.h"

// Ganti SSID dan password ini sebelum upload.
#ifndef TELEMETRY_WIFI_SSID
#define TELEMETRY_WIFI_SSID GCS_WIFI_SSID
#endif

#ifndef TELEMETRY_WIFI_PASSWORD
#define TELEMETRY_WIFI_PASSWORD GCS_WIFI_PASSWORD
#endif

#define TELEMETRY_HOSTNAME GCS_MDNS_HOSTNAME
#define TELEMETRY_HTTP_PORT 8080
#define TELEMETRY_WIFI_TIMEOUT_MS 10000
#define TELEMETRY_TASK_PERIOD_MS 10

WebServer telemetryServer(TELEMETRY_HTTP_PORT);
bool telemetry_wifi_ready = false;
bool telemetry_server_ready = false;

String telemetry_json() {
    char buffer[512];

    snprintf(buffer, sizeof(buffer),
        "{"
        "\"time_ms\":%lu,"
        "\"radio_ok\":%d,"
        "\"failsafe\":%d,"
        "\"arm\":%d,"
        "\"mode\":%d,"
        "\"roll\":%.2f,"
        "\"pitch\":%.2f,"
        "\"yaw\":%.2f,"
        "\"gx\":%.2f,"
        "\"gy\":%.2f,"
        "\"gz\":%.2f,"
        "\"ch_roll\":%d,"
        "\"ch_pitch\":%d,"
        "\"ch_throttle\":%d,"
        "\"ch_yaw\":%d,"
        "\"motor1\":%d,"
        "\"motor2\":%d,"
        "\"motor3\":%d,"
        "\"motor4\":%d,"
        "\"control1\":%d,"
        "\"control2\":%d,"
        "\"control3\":%d,"
        "\"control4\":%d"
        "}",
        millis(),
        radio_frame_valid ? 1 : 0,
        (radio_failsafe || signal_lost) ? 1 : 0,
        arming ? 1 : 0,
        mode_now,
        roll,
        pitch,
        yaw,
        gx,
        gy,
        gz,
        ch_roll,
        ch_pitch,
        ch_throttle,
        ch_yaw,
        (int)m1_pwm,
        (int)m2_pwm,
        (int)m3_pwm,
        (int)m4_pwm,
        control1,
        control2,
        control3,
        control4
    );

    return String(buffer);
}

String telemetry_csv() {
    char buffer[320];

    snprintf(buffer, sizeof(buffer),
        "%lu,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
        millis(),
        radio_frame_valid ? 1 : 0,
        (radio_failsafe || signal_lost) ? 1 : 0,
        arming ? 1 : 0,
        mode_now,
        roll,
        pitch,
        yaw,
        gx,
        gy,
        gz,
        ch_roll,
        ch_pitch,
        ch_throttle,
        ch_yaw,
        (int)m1_pwm,
        (int)m2_pwm,
        (int)m3_pwm,
        (int)m4_pwm,
        control1,
        control2,
        control3,
        control4
    );

    return String(buffer);
}

void telemetry_send_cors() {
    telemetryServer.sendHeader("Access-Control-Allow-Origin", "*");
    telemetryServer.sendHeader("Cache-Control", "no-store");
}

void telemetry_handle_root() {
    telemetry_send_cors();
    telemetryServer.send(200, "text/html",
        "<!doctype html>"
        "<html>"
        "<head>"
        "<meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<title>Proyek Robotika Lanjut Telemetry</title>"
        "<style>"
        "body{font-family:Arial,sans-serif;margin:24px;background:#111;color:#eee}"
        "h1{font-size:22px}"
        "pre{font-size:14px;line-height:1.5;white-space:pre-wrap}"
        ".ok{color:#47d16c}.bad{color:#ff5b5b}"
        "</style>"
        "</head>"
        "<body>"
        "<h1>Proyek Robotika Lanjut Telemetry</h1>"
        "<pre id='data'>connecting...</pre>"
        "<script>"
        "async function refresh(){"
        "try{"
        "const r=await fetch('/telemetry');"
        "const j=await r.json();"
        "document.getElementById('data').textContent=JSON.stringify(j,null,2);"
        "}catch(e){document.getElementById('data').textContent='telemetry error: '+e;}"
        "}"
        "refresh();setInterval(refresh,200);"
        "</script>"
        "</body>"
        "</html>"
    );
}

void telemetry_handle_json() {
    telemetry_send_cors();
    telemetryServer.send(200, "application/json", telemetry_json());
}

void telemetry_handle_csv() {
    telemetry_send_cors();
    telemetryServer.send(200, "text/plain", telemetry_csv());
}

void telemetry_handle_status() {
    telemetry_send_cors();
    telemetryServer.send(200, "text/plain", telemetry_wifi_ready ? "ready" : "wifi_not_ready");
}

void telemetry_setup_routes() {
    telemetryServer.on("/", HTTP_GET, telemetry_handle_root);
    telemetryServer.on("/telemetry", HTTP_GET, telemetry_handle_json);
    telemetryServer.on("/telemetry.csv", HTTP_GET, telemetry_handle_csv);
    telemetryServer.on("/status", HTTP_GET, telemetry_handle_status);
}

void telemetry_setup() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(TELEMETRY_WIFI_SSID, TELEMETRY_WIFI_PASSWORD);

    Serial.print("Telemetry WiFi connecting");
    unsigned long start_ms = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start_ms) < TELEMETRY_WIFI_TIMEOUT_MS) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Telemetry WiFi failed");
        telemetry_wifi_ready = false;
        return;
    }

    telemetry_wifi_ready = true;
    Serial.print("Telemetry WiFi IP: ");
    Serial.println(WiFi.localIP());

    if (MDNS.begin(TELEMETRY_HOSTNAME)) {
        MDNS.addService("http", "tcp", TELEMETRY_HTTP_PORT);
        Serial.print("Telemetry mDNS: http://");
        Serial.print(TELEMETRY_HOSTNAME);
        Serial.print(".local:");
        Serial.println(TELEMETRY_HTTP_PORT);
    } else {
        Serial.println("Telemetry mDNS failed, use ESP32 IP address");
    }

    telemetry_setup_routes();
    telemetryServer.begin();
    telemetry_server_ready = true;

    Serial.println("Telemetry server started");
}

void telemetry_loop() {
    if (!telemetry_server_ready) {
        return;
    }

    telemetryServer.handleClient();
}

void telemetryTask(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        telemetry_loop();
        vTaskDelay(pdMS_TO_TICKS(TELEMETRY_TASK_PERIOD_MS));
    }
}
