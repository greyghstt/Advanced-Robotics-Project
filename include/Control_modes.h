//update mode
#pragma once
#include <Arduino.h>
#include "Radio.h"
#include "Ultrasonik.h"

// bool altitude_hold_mode = false;
// bool position_hold_mode = false;
// bool pos_state = false;

float roll_setpoint;
float pitch_setpoint;
float yaw_setpoint;
float alt_setpoint;
int32_t setpoint_lat, setpoint_lon, last_lat, last_lon, error_lat, error_lon;
// float last_alt;
bool start;
// int16_t nav_roll;
// int16_t nav_pitch;
// int16_t nav_yaw;

void AltHoldUpdate() //full nembak pwm
{
  if (ch_throttle > 1600) {
    alt_setpoint = altitude;
    alt_setpoint += (ch_throttle-1600)/20000;
  }
  else if (ch_throttle < 1400) {
    alt_setpoint = altitude;
    alt_setpoint -= (ch_throttle-1400)/20000;
  }
  else {
    alt_setpoint = altitude;
  }
}

//full nembak pwm
// void PosHoldUpdate()
// {
//   if (ch_roll == 1500 && ch_pitch == 1500)
//   {
//     setpoint_lat = latitude;
//     setpoint_lon = longitude;
//     error_lat =  (setpoint_lat - last_lat)*1000000;
//     error_lon = (setpoint_lon - last_lon)*1000000;
//     error_lat = ((float)error_lat * cos(heading * 0.017453)) + ((float)error_lon * cos(heading - 90) * 0.017453);
//     error_lon = ((float)error_lon * cos(heading * 0.017453)) + ((float)error_lat * cos(heading - 90) * 0.017453);
    
//     nav_roll = constrain(error_lon, -200, 200);
//     nav_pitch = constrain(error_lat, -200, 200);
//   }
//   else
//   {
//     error_lat = 0;
//     error_lon = 0;
//   }
// }

void update_mode()
{
    if(alt_hold){
        AltHoldUpdate();
    }
    else if (pos_hold){
        // PosHoldUpdate();
    }
}