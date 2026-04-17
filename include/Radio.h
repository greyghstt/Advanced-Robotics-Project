#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>
#include <HardwareSerial.h>

HardwareSerial SerialSBUS(2); // UART2: RX=GPIO16, TX=GPIO17

#define SBUS_PACKET_SIZE 25
uint8_t sbusData[SBUS_PACKET_SIZE];

int16_t channels[16];

bool signal_lost;
int16_t arm, ch_roll, ch_pitch, ch_throttle, ch_yaw, ch_mode, ch_mode_backup, ch_vehicle_mode;
bool arming;

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

int mode_now, prev_mode;

float outputScaler(uint16_t ch) {
  return 0.002 * ch - 3;  // Skala dari 172-1811 jadi ~0-1
}

void remote_setup() {
  Serial.begin(115200);
  SerialSBUS.begin(100000, SERIAL_8E2, 16, 17); 
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
    signal_lost = sbusData[23] & 0x04;  // Bit ke-2 (0x04) menunjukkan signal lost
    bool failsafe = sbusData[23] & 0x08; // Bit ke-3 (0x08) menunjukkan failsafe

    // Validasi byte akhir (opsional, bisa disesuaikan)
    if (sbusData[24] != 0x00) {
      continue;
    }

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

    bool valid = true;
    for (int i = 0; i < 16; i++) {
        if (channels[i] < 172 || channels[i] > 1810) {
            valid = false;
            break;
        }
    }
    if (!valid) {
        Serial.println("Invalid SBUS frame, skipping...");
        return;
    }

    // // Peringatan jika data aneh
    // if (channels[0] < 100 || channels[0] > 2000) {
    //   Serial.println("⚠️  Warning: Channel data out of range!");
    // }

    // Konversi ke PWM (1000–2000 µs)
    ch_roll     = constrain(map(channels[0], 172, 1810, 1000, 2000), 1000, 2000);
    ch_pitch    = constrain(map(channels[1], 172, 1810, 1000, 2000), 1000, 2000);
    ch_throttle = constrain(map(channels[2], 172, 1810, 1000, 2000), 1000, 2000);
    ch_yaw      = constrain(map(channels[3], 172, 1810, 1000, 2000), 1000, 2000);
    arm         = constrain(map(channels[4], 172, 1810, 1000, 2000), 1000, 2000);

    arming = arm > 1500 ? 1 : 0;
    signal_lost = signal_lost;

    ch_mode         = map(channels[5], 172, 1810, 1000, 2000);
    ch_mode_backup  = map(channels[6], 172, 1810, 1000, 2000);
    ch_vehicle_mode = map(channels[7], 172, 1810, 1000, 2000);

    arming = arm > 1500 ? 1 : 0;
    signal_lost = signal_lost; // This line seems redundant and doesn't make sense.
    // Serial.print("Roll PWM: "); Serial.println(ch_roll);  
    // Serial.print("Pitch PWM: "); Serial.println(ch_pitch);
    // Serial.print("Throttle PWM: "); Serial.println(ch_throttle);
    // Serial.print("Yaw PWM: "); Serial.println(ch_yaw);
    
    // Penentuan mode
    if (ch_mode <= 1000) {
      mode_now = 1;
    } else if (ch_mode <= 1512) {
      mode_now = 2;
    } else {
      mode_now = 3;
    }
  }
}

#endif
