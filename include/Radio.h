#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial SerialSBUS(2); // UART2: RX=GPIO16, TX=GPIO17

#define SBUS_PACKET_SIZE 25
#define SBUS_MIN_RAW 172
#define SBUS_MAX_RAW 1810
#define RADIO_PWM_MIN 1000
#define RADIO_PWM_MID 1500
#define RADIO_PWM_MAX 2000
uint8_t sbusData[SBUS_PACKET_SIZE];

int16_t channels[16];

bool signal_lost = true;
bool radio_failsafe = true;
bool radio_frame_valid = false;
uint32_t last_radio_frame_ms = 0;
int16_t arm = RADIO_PWM_MIN;
int16_t ch_roll = RADIO_PWM_MID;
int16_t ch_pitch = RADIO_PWM_MID;
int16_t ch_throttle = RADIO_PWM_MIN;
int16_t ch_yaw = RADIO_PWM_MID;
int16_t ch_mode = RADIO_PWM_MIN;
int16_t ch_mode_backup = RADIO_PWM_MIN;
int16_t ch_vehicle_mode = RADIO_PWM_MIN;
bool arming = false;

bool alt_hold = false;
bool mode_fbwa = false;
bool pos_hold = false;
bool mode_manual = false;
bool mode_fbwa_plane = false;
bool mode_fbwb = false;
bool transition_phase1 = false;
bool transition_phase2 = false;
bool transition_phase3 = false;
bool mode_vtol = false;
bool mode_safety = false;
bool mode_vtol_plane = false;

int mode_now = 1, prev_mode = 1;

float outputScaler(uint16_t ch) {
  return 0.002 * ch - 3;  // Skala dari 172-1811 jadi ~0-1
}

bool sbus_channel_valid(int16_t value) {
  return value >= SBUS_MIN_RAW && value <= SBUS_MAX_RAW;
}

int16_t sbus_to_pwm(int16_t value, int16_t fallback_pwm) {
  if (!sbus_channel_valid(value)) {
    return fallback_pwm;
  }

  return constrain(map(value, SBUS_MIN_RAW, SBUS_MAX_RAW, RADIO_PWM_MIN, RADIO_PWM_MAX),
                   RADIO_PWM_MIN, RADIO_PWM_MAX);
}

void radio_set_safe_output() {
  ch_roll = RADIO_PWM_MID;
  ch_pitch = RADIO_PWM_MID;
  ch_throttle = RADIO_PWM_MIN;
  ch_yaw = RADIO_PWM_MID;
  arm = RADIO_PWM_MIN;
  ch_mode = RADIO_PWM_MIN;
  ch_mode_backup = RADIO_PWM_MIN;
  ch_vehicle_mode = RADIO_PWM_MIN;
  arming = false;
  mode_now = 1;
  radio_frame_valid = false;
}

void remote_setup() {
  Serial.begin(115200);
  radio_set_safe_output();
  SerialSBUS.begin(100000, SERIAL_8E2, 35, 17);
  SerialSBUS.setRxInvert(true);
}

void remote_loop() {
  // Serial.println("Remote loop running");
  while (SerialSBUS.available() >= SBUS_PACKET_SIZE) {
    // Sinkronisasi SBUS: cari byte awal 0x0F
    if (SerialSBUS.peek() != 0x0F) {
      SerialSBUS.read(); // buang byte tidak valid
      continue;
    }
   
    SerialSBUS.readBytes(sbusData, SBUS_PACKET_SIZE);

    // Periksa flag signal lost dan failsafe di byte akhir SBUS
    signal_lost = (sbusData[23] & 0x04) != 0;  // Bit ke-2 (0x04) menunjukkan signal lost
    radio_failsafe = (sbusData[23] & 0x08) != 0; // Bit ke-3 (0x08) menunjukkan failsafe

    // Validasi byte akhir (opsional, bisa disesuaikan)
    // if (sbusData[24] != 0x00) {
    //   continue;
    // }

    // Decode channel SBUS
    channels[0]  = ((sbusData[1]    | sbusData[2]  << 8) & 0x07FF);
    channels[1]  = ((sbusData[2] >> 3 | sbusData[3] << 5) & 0x07FF);
    channels[2]  = ((sbusData[3] >> 6 | sbusData[4] << 2 | sbusData[5] << 10) & 0x07FF);
    channels[3]  = ((sbusData[5] >> 1 | sbusData[6] << 7) & 0x07FF);
    channels[4]  = ((sbusData[6] >> 4 | sbusData[7] << 4) & 0x07FF);
    channels[5]  = ((sbusData[7] >> 7 | sbusData[8] << 1 | sbusData[9] << 9) & 0x07FF);
    channels[6]  = ((sbusData[9] >> 2 | sbusData[10] << 6) & 0x07FF);
    channels[7]  = ((sbusData[10] >> 5 | sbusData[11] << 3) & 0x07FF);
    channels[8]  = ((sbusData[12]    | sbusData[13] << 8) & 0x07FF);
    channels[9]  = ((sbusData[13] >> 3 | sbusData[14] << 5) & 0x07FF);
    channels[10] = ((sbusData[14] >> 6 | sbusData[15] << 2 | sbusData[16] << 10) & 0x07FF);
    channels[11] = ((sbusData[16] >> 1 | sbusData[17] << 7) & 0x07FF);
    channels[12] = ((sbusData[17] >> 4 | sbusData[18] << 4) & 0x07FF);
    channels[13] = ((sbusData[18] >> 7 | sbusData[19] << 1 | sbusData[20] << 9) & 0x07FF);
    channels[14] = ((sbusData[20] >> 2 | sbusData[21] << 6) & 0x07FF);
    channels[15] = ((sbusData[21] >> 5 | sbusData[22] << 3) & 0x07FF);

    if (signal_lost || radio_failsafe) {
      radio_set_safe_output();
      Serial.println("SBUS failsafe/signal lost, motor output forced to minimum");
      continue;
    }

    bool control_channels_valid = true;
    for (int i = 0; i < 4; i++) {
      if (!sbus_channel_valid(channels[i])) {
        control_channels_valid = false;
        break;
      }
    }

    if (!control_channels_valid) {
      radio_set_safe_output();
      Serial.println("Invalid SBUS control channel, motor output forced to minimum");
      continue;
    }

    // // Peringatan jika data aneh
    // if (channels[0] < 100 || channels[0] > 2000) {
    //   Serial.println("⚠️  Warning: Channel data out of range!");
    // }

    // Konversi ke PWM (1000–2000 µs)
    ch_roll     = sbus_to_pwm(channels[0], RADIO_PWM_MID);
    ch_pitch    = sbus_to_pwm(channels[1], RADIO_PWM_MID);
    ch_throttle = sbus_to_pwm(channels[2], RADIO_PWM_MIN);
    ch_yaw      = sbus_to_pwm(channels[3], RADIO_PWM_MID);
    arm         = sbus_to_pwm(channels[4], RADIO_PWM_MIN);

    ch_mode         = sbus_to_pwm(channels[5], RADIO_PWM_MIN);
    ch_mode_backup  = sbus_to_pwm(channels[6], RADIO_PWM_MIN);
    ch_vehicle_mode = sbus_to_pwm(channels[7], RADIO_PWM_MIN);

    arming = arm > RADIO_PWM_MID;
    radio_frame_valid = true;
    last_radio_frame_ms = millis();
    // Serial.print("Roll PWM: "); Serial.println(ch_roll);  
    // Serial.print("Pitch PWM: "); Serial.println(ch_pitch);
    // Serial.print("Throttle PWM: "); Serial.println(ch_throttle);
    // Serial.print("Yaw PWM: "); Serial.println(ch_yaw);
    
    // Penentuan mode
    prev_mode = mode_now;
    if (ch_mode < 1250) {
      mode_now = 1;
    } else if (ch_mode < 1750) {
      mode_now = 2;
    } else {
      mode_now = 3;
    }
  }
}

#endif
