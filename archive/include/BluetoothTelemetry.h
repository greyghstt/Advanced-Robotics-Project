#pragma once

// System A Bluetooth GCS archive.
// UDP GCS is the active mode, so this file is intentionally not compiled.
#if 0

#include <Arduino.h>
#include <BluetoothSerial.h>

#include "GcsConfig.h"
#include "Radio.h"
#include "Actuator.h"
#include "ImuAcquisition.h"

#define BT_GCS_DEVICE_NAME "Robjut-GCS"
#define BT_GCS_TASK_PERIOD_MS 20
#define BT_GCS_SEND_PERIOD_MS 100
#ifndef BT_GCS_ALLOW_ARMED_PID_UPDATE
#define BT_GCS_ALLOW_ARMED_PID_UPDATE 0
#endif

BluetoothSerial SerialBT;
bool bt_gcs_ready = false;
unsigned long bt_gcs_last_send_ms = 0;

const char BT_GCS_CSV_HEADER[] =
    "type,time_ms,radio_ok,failsafe,arm,mode,"
    "roll,pitch,yaw,gx,gy,gz,speed_z,"
    "ch_roll,ch_pitch,ch_throttle,ch_yaw,"
    "motor1,motor2,motor3,motor4,"
    "control1,control2,control3,control4,"
    "k_roll,k_pitch,k_yaw,k_roll_rate,k_pitch_rate,k_yaw_rate,"
    "k_i_roll,k_i_pitch,k_i_yaw";

float bt_gcs_constrain_float(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

bool bt_gcs_parse_float(const String &text, float *value) {
    char *end_ptr = nullptr;
    float parsed = strtof(text.c_str(), &end_ptr);

    if (end_ptr == text.c_str() || *end_ptr != '\0') {
        return false;
    }

    *value = parsed;
    return true;
}

String bt_gcs_telemetry_csv() {
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

String bt_gcs_telemetry_json() {
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

bool bt_gcs_set_gain(const String &raw_name, float value, String *message) {
    if (arming && !BT_GCS_ALLOW_ARMED_PID_UPDATE) {
        *message = "ERR,PID_UPDATE_BLOCKED_WHILE_ARMED";
        return false;
    }

    String name = raw_name;
    name.trim();
    name.toUpperCase();

    if (name == "K_ROLL") {
        gain.k_roll = bt_gcs_constrain_float(value, 0.0f, 20.0f);
    } else if (name == "K_PITCH") {
        gain.k_pitch = bt_gcs_constrain_float(value, 0.0f, 20.0f);
    } else if (name == "K_YAW") {
        gain.k_yaw = bt_gcs_constrain_float(value, 0.0f, 20.0f);
    } else if (name == "K_ROLL_RATE") {
        gain.k_roll_rate = bt_gcs_constrain_float(value, 0.0f, 10.0f);
    } else if (name == "K_PITCH_RATE") {
        gain.k_pitch_rate = bt_gcs_constrain_float(value, 0.0f, 10.0f);
    } else if (name == "K_YAW_RATE") {
        gain.k_yaw_rate = bt_gcs_constrain_float(value, 0.0f, 10.0f);
    } else if (name == "K_I_ROLL") {
        gain.k_i_roll = bt_gcs_constrain_float(value, 0.0f, 2.0f);
    } else if (name == "K_I_PITCH") {
        gain.k_i_pitch = bt_gcs_constrain_float(value, 0.0f, 2.0f);
    } else if (name == "K_I_YAW") {
        gain.k_i_yaw = bt_gcs_constrain_float(value, 0.0f, 2.0f);
    } else if (name == "TRIM_ROLL") {
        trim_roll = bt_gcs_constrain_float(value, -20.0f, 20.0f);
    } else if (name == "TRIM_PITCH") {
        trim_pitch = bt_gcs_constrain_float(value, -20.0f, 20.0f);
    } else if (name == "TRIM_YAW") {
        trim_yaw = bt_gcs_constrain_float(value, -20.0f, 20.0f);
    } else {
        *message = "ERR,UNKNOWN_PID_NAME";
        return false;
    }

    *message = "ACK,SET," + name + "," + String(value, 4);
    return true;
}

void bt_gcs_send_help() {
    SerialBT.println("HELP,COMMANDS: HEADER | GET CSV | GET JSON | PID | SET K_PITCH 4.8");
    SerialBT.println("HELP,PID_NAMES: K_ROLL,K_PITCH,K_YAW,K_ROLL_RATE,K_PITCH_RATE,K_YAW_RATE,K_I_ROLL,K_I_PITCH,K_I_YAW");
}

void bt_gcs_send_pid() {
    SerialBT.print("PID,");
    SerialBT.print(gain.k_roll, 4); SerialBT.print(",");
    SerialBT.print(gain.k_pitch, 4); SerialBT.print(",");
    SerialBT.print(gain.k_yaw, 4); SerialBT.print(",");
    SerialBT.print(gain.k_roll_rate, 4); SerialBT.print(",");
    SerialBT.print(gain.k_pitch_rate, 4); SerialBT.print(",");
    SerialBT.print(gain.k_yaw_rate, 4); SerialBT.print(",");
    SerialBT.print(gain.k_i_roll, 4); SerialBT.print(",");
    SerialBT.print(gain.k_i_pitch, 4); SerialBT.print(",");
    SerialBT.println(gain.k_i_yaw, 4);
}

void bt_gcs_handle_command(String line) {
    line.trim();
    if (line.length() == 0) return;

    String upper = line;
    upper.toUpperCase();

    if (upper == "PING") {
        SerialBT.println("PONG");
    } else if (upper == "HELP") {
        bt_gcs_send_help();
    } else if (upper == "HEADER") {
        SerialBT.println(BT_GCS_CSV_HEADER);
    } else if (upper == "GET CSV") {
        SerialBT.println(bt_gcs_telemetry_csv());
    } else if (upper == "GET JSON") {
        SerialBT.println(bt_gcs_telemetry_json());
    } else if (upper == "PID") {
        bt_gcs_send_pid();
    } else if (upper.startsWith("SET ")) {
        String payload = line.substring(4);
        payload.trim();

        int separator = payload.indexOf(' ');
        if (separator < 0) {
            separator = payload.indexOf('=');
        }

        if (separator < 0) {
            SerialBT.println("ERR,SET_FORMAT");
            return;
        }

        String name = payload.substring(0, separator);
        String value_text = payload.substring(separator + 1);
        name.trim();
        value_text.trim();

        float value = 0.0f;
        if (!bt_gcs_parse_float(value_text, &value)) {
            SerialBT.println("ERR,INVALID_VALUE");
            return;
        }

        String message;
        bt_gcs_set_gain(name, value, &message);
        SerialBT.println(message);
    } else {
        SerialBT.println("ERR,UNKNOWN_COMMAND");
    }
}

void bt_gcs_read_commands() {
    while (SerialBT.available()) {
        String line = SerialBT.readStringUntil('\n');
        bt_gcs_handle_command(line);
    }
}

void bt_gcs_send_telemetry() {
    if (!SerialBT.hasClient()) {
        return;
    }

    unsigned long now_ms = millis();
    if ((now_ms - bt_gcs_last_send_ms) < BT_GCS_SEND_PERIOD_MS) {
        return;
    }

    bt_gcs_last_send_ms = now_ms;
    SerialBT.println(bt_gcs_telemetry_csv());
}

void bt_gcs_setup() {
    if (!SerialBT.begin(BT_GCS_DEVICE_NAME)) {
        Serial.println("Bluetooth GCS start failed");
        bt_gcs_ready = false;
        return;
    }

    SerialBT.setTimeout(5);
    bt_gcs_ready = true;
    Serial.print("Bluetooth GCS ready: ");
    Serial.println(BT_GCS_DEVICE_NAME);
}

void bt_gcs_loop() {
    if (!bt_gcs_ready) {
        return;
    }

    bt_gcs_read_commands();
    bt_gcs_send_telemetry();
}

void btGcsTask(void *pvParameters) {
    (void)pvParameters;

    for (;;) {
        bt_gcs_loop();
        vTaskDelay(pdMS_TO_TICKS(BT_GCS_TASK_PERIOD_MS));
    }
}

#endif // System A Bluetooth GCS archive
