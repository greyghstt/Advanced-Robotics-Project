#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ESPmDNS.h>

#include "Gcs_config.h"
#include "Radio.h"
#include "Actuator.h"
#include "akuisisi.h"

WiFiUDP udpGcs;
bool udp_gcs_wifi_ready = false;
bool udp_gcs_ready = false;
bool udp_gcs_client_known = false;
IPAddress udp_gcs_client_ip;
uint16_t udp_gcs_client_port = 0;
unsigned long udp_gcs_last_send_ms = 0;

const char UDP_GCS_CSV_HEADER[] =
    "type,time_ms,radio_ok,failsafe,arm,mode,"
    "roll,pitch,yaw,gx,gy,gz,speed_z,"
    "ch_roll,ch_pitch,ch_throttle,ch_yaw,"
    "motor1,motor2,motor3,motor4,"
    "control1,control2,control3,control4,"
    "k_roll,k_pitch,k_yaw,k_roll_rate,k_pitch_rate,k_yaw_rate,"
    "k_i_roll,k_i_pitch,k_i_yaw";

float udp_gcs_constrain_float(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

bool udp_gcs_parse_float(const String &text, float *value) {
    char *end_ptr = nullptr;
    float parsed = strtof(text.c_str(), &end_ptr);

    if (end_ptr == text.c_str() || *end_ptr != '\0') {
        return false;
    }

    *value = parsed;
    return true;
}

String udp_gcs_telemetry_csv() {
    char buffer[768];

    snprintf(buffer, sizeof(buffer),
        "TEL,%lu,%d,%d,%d,%d,"
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
        "%d,%d,%d,%d,"
        "%d,%d,%d,%d,"
        "%d,%d,%d,%d,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
        "%.4f,%.4f,%.4f",
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
        z_velocity,
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
        control4,
        gain.k_roll,
        gain.k_pitch,
        gain.k_yaw,
        gain.k_roll_rate,
        gain.k_pitch_rate,
        gain.k_yaw_rate,
        gain.k_i_roll,
        gain.k_i_pitch,
        gain.k_i_yaw
    );

    return String(buffer);
}

String udp_gcs_telemetry_json() {
    char buffer[1024];

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
        "\"speed_z\":%.2f,"
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
        "\"control4\":%d,"
        "\"k_roll\":%.4f,"
        "\"k_pitch\":%.4f,"
        "\"k_yaw\":%.4f,"
        "\"k_roll_rate\":%.4f,"
        "\"k_pitch_rate\":%.4f,"
        "\"k_yaw_rate\":%.4f,"
        "\"k_i_roll\":%.4f,"
        "\"k_i_pitch\":%.4f,"
        "\"k_i_yaw\":%.4f"
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
        z_velocity,
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
        control4,
        gain.k_roll,
        gain.k_pitch,
        gain.k_yaw,
        gain.k_roll_rate,
        gain.k_pitch_rate,
        gain.k_yaw_rate,
        gain.k_i_roll,
        gain.k_i_pitch,
        gain.k_i_yaw
    );

    return String(buffer);
}

void udp_gcs_send_line(const String &line) {
    if (!udp_gcs_client_known) {
        return;
    }

    udpGcs.beginPacket(udp_gcs_client_ip, udp_gcs_client_port);
    udpGcs.write((const uint8_t *)line.c_str(), line.length());
    udpGcs.endPacket();
}

bool udp_gcs_set_gain(const String &raw_name, float value, String *message) {
    if (arming && !UDP_GCS_ALLOW_ARMED_PID_UPDATE) {
        *message = "ERR,PID_UPDATE_BLOCKED_WHILE_ARMED";
        return false;
    }

    String name = raw_name;
    name.trim();
    name.toUpperCase();

    if (name == "K_ROLL") {
        gain.k_roll = udp_gcs_constrain_float(value, 0.0f, 20.0f);
    } else if (name == "K_PITCH") {
        gain.k_pitch = udp_gcs_constrain_float(value, 0.0f, 20.0f);
    } else if (name == "K_YAW") {
        gain.k_yaw = udp_gcs_constrain_float(value, 0.0f, 20.0f);
    } else if (name == "K_ROLL_RATE") {
        gain.k_roll_rate = udp_gcs_constrain_float(value, 0.0f, 10.0f);
    } else if (name == "K_PITCH_RATE") {
        gain.k_pitch_rate = udp_gcs_constrain_float(value, 0.0f, 10.0f);
    } else if (name == "K_YAW_RATE") {
        gain.k_yaw_rate = udp_gcs_constrain_float(value, 0.0f, 10.0f);
    } else if (name == "K_I_ROLL") {
        gain.k_i_roll = udp_gcs_constrain_float(value, 0.0f, 2.0f);
    } else if (name == "K_I_PITCH") {
        gain.k_i_pitch = udp_gcs_constrain_float(value, 0.0f, 2.0f);
    } else if (name == "K_I_YAW") {
        gain.k_i_yaw = udp_gcs_constrain_float(value, 0.0f, 2.0f);
    } else if (name == "TRIM_ROLL") {
        trim_roll = udp_gcs_constrain_float(value, -20.0f, 20.0f);
    } else if (name == "TRIM_PITCH") {
        trim_pitch = udp_gcs_constrain_float(value, -20.0f, 20.0f);
    } else if (name == "TRIM_YAW") {
        trim_yaw = udp_gcs_constrain_float(value, -20.0f, 20.0f);
    } else {
        *message = "ERR,UNKNOWN_PID_NAME";
        return false;
    }

    *message = "ACK,SET," + name + "," + String(value, 4);
    return true;
}

void udp_gcs_send_pid() {
    char buffer[192];
    snprintf(buffer, sizeof(buffer),
        "PID,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
        gain.k_roll,
        gain.k_pitch,
        gain.k_yaw,
        gain.k_roll_rate,
        gain.k_pitch_rate,
        gain.k_yaw_rate,
        gain.k_i_roll,
        gain.k_i_pitch,
        gain.k_i_yaw
    );
    udp_gcs_send_line(String(buffer));
}

void udp_gcs_handle_command(String line) {
    line.trim();
    if (line.length() == 0) return;

    String upper = line;
    upper.toUpperCase();

    if (upper == "HELLO" || upper == "PING") {
        udp_gcs_send_line("ACK,HELLO");
        udp_gcs_send_line(UDP_GCS_CSV_HEADER);
    } else if (upper == "HELP") {
        udp_gcs_send_line("HELP,COMMANDS: HELLO | HEADER | GET CSV | GET JSON | PID | SET K_PITCH 4.8");
    } else if (upper == "HEADER") {
        udp_gcs_send_line(UDP_GCS_CSV_HEADER);
    } else if (upper == "GET CSV") {
        udp_gcs_send_line(udp_gcs_telemetry_csv());
    } else if (upper == "GET JSON") {
        udp_gcs_send_line(udp_gcs_telemetry_json());
    } else if (upper == "PID") {
        udp_gcs_send_pid();
    } else if (upper.startsWith("SET ")) {
        String payload = line.substring(4);
        payload.trim();

        int separator = payload.indexOf(' ');
        if (separator < 0) {
            separator = payload.indexOf('=');
        }

        if (separator < 0) {
            udp_gcs_send_line("ERR,SET_FORMAT");
            return;
        }

        String name = payload.substring(0, separator);
        String value_text = payload.substring(separator + 1);
        name.trim();
        value_text.trim();

        float value = 0.0f;
        if (!udp_gcs_parse_float(value_text, &value)) {
            udp_gcs_send_line("ERR,INVALID_VALUE");
            return;
        }

        String message;
        udp_gcs_set_gain(name, value, &message);
        udp_gcs_send_line(message);
    } else {
        udp_gcs_send_line("ERR,UNKNOWN_COMMAND");
    }
}

void udp_gcs_read_packets() {
    int packet_size = udpGcs.parsePacket();
    if (packet_size <= 0) {
        return;
    }

    udp_gcs_client_ip = udpGcs.remoteIP();
    udp_gcs_client_port = udpGcs.remotePort();
    udp_gcs_client_known = true;

    char packet[256];
    int len = udpGcs.read(packet, sizeof(packet) - 1);
    if (len <= 0) {
        return;
    }

    packet[len] = '\0';
    udp_gcs_handle_command(String(packet));
}

void udp_gcs_send_telemetry() {
    if (!udp_gcs_client_known) {
        return;
    }

    unsigned long now_ms = millis();
    if ((now_ms - udp_gcs_last_send_ms) < UDP_GCS_SEND_PERIOD_MS) {
        return;
    }

    udp_gcs_last_send_ms = now_ms;
    udp_gcs_send_line(udp_gcs_telemetry_csv());
}

void udp_gcs_setup() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);

    if (WiFi.status() != WL_CONNECTED) {
        WiFi.begin(GCS_WIFI_SSID, GCS_WIFI_PASSWORD);

        Serial.print("UDP GCS WiFi connecting");
        unsigned long start_ms = millis();
        while (WiFi.status() != WL_CONNECTED && (millis() - start_ms) < UDP_GCS_WIFI_TIMEOUT_MS) {
            delay(250);
            Serial.print(".");
        }
        Serial.println();
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("UDP GCS WiFi failed");
        udp_gcs_wifi_ready = false;
        return;
    }

    udp_gcs_wifi_ready = true;
    Serial.print("UDP GCS WiFi IP: ");
    Serial.println(WiFi.localIP());

    if (MDNS.begin(GCS_MDNS_HOSTNAME)) {
        MDNS.addService("robjut-udp", "udp", UDP_GCS_LISTEN_PORT);
        Serial.print("UDP GCS mDNS: ");
        Serial.print(GCS_MDNS_HOSTNAME);
        Serial.print(".local:");
        Serial.println(UDP_GCS_LISTEN_PORT);
    } else {
        Serial.println("UDP GCS mDNS failed, use ESP32 IP address");
    }

    udp_gcs_ready = udpGcs.begin(UDP_GCS_LISTEN_PORT);
    Serial.print("UDP GCS listen port: ");
    Serial.println(UDP_GCS_LISTEN_PORT);
}

void udp_gcs_loop() {
    if (!udp_gcs_ready) {
        return;
    }

    udp_gcs_read_packets();
    udp_gcs_send_telemetry();
}

void udpGcsTask(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        udp_gcs_loop();
        vTaskDelay(pdMS_TO_TICKS(UDP_GCS_TASK_PERIOD_MS));
    }
}
