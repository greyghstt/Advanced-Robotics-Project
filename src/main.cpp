#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <Kalman.h>
#include <Radio.h>
#include <Ultrasonik.h>
#include <Actuator.h>
#include <akuisisi.h>
#include <Transisition.h>
#include <Copter_control.h>

#define IMU_time 5
#define Ultrasonik_time 10
#define PRINT_time 50
#define CONTROL_time 5
#define RADIO_time 10

TaskHandle_t Task_IMU;
TaskHandle_t Task_Ultrasonik;
TaskHandle_t Task_Print;
TaskHandle_t Task_Radio; // Tambahkan handle untuk task radio

void printUSB() {
  // Read roll, pitch, yaw, and throttle channels
  // Serial.print("omega2[0]: "); Serial.print(omega2[0]);
  // Serial.print("omega2[1]: "); Serial.print(omega2[1]);
  // Serial.print("omega2[2]: "); Serial.print(omega2[2]);
  // Serial.print("omega2[3]: "); Serial.print(omega2[3]);
  // Serial.print("Control1: "); Serial.print(control1);
  // Serial.print("Control2: "); Serial.print(control2);
  // Serial.print("Control3: "); Serial.print(control3);
  // Serial.print("Control4: "); Serial.print(control4);
  // Serial.print("arm: "); Serial.print(arming);
  Serial.print(" rol: "); Serial.print(roll);
  Serial.print(" pit: "); Serial.print(pitch);
  Serial.print(" yw: "); Serial.print(yaw);
  // Serial.print(" m4_pwm: "); Serial.print(m4_pwm);
//print pwm motor
  // Serial.print("arm: "); Serial.print(arming);
  // Serial.print(" m1_pwm: "); Serial.print(m1_pwm);
  // Serial.print(" m2_pwm: "); Serial.print(m2_pwm);
  // Serial.print(" m3_pwm: "); Serial.print(m3_pwm);
  // Serial.print(" m4_pwm: "); Serial.print(m4_pwm);
  // Serial.print("ch_mode_backup : "); Serial.print(ch_mode_backup);

  // for (int i = 0; i < 8; i++) {
  //   Serial.print("CH"); Serial.print(i); Serial.print(": ");
  //   Serial.print(channels[i]); Serial.print("\t");
  // }

  Serial.println();
}

void updateIMU(void *pvParameters){ 
  for(;;){
      ambil_data_imu();
      vTaskDelay(pdMS_TO_TICKS(IMU_time));
  }
};

void updateUltrasonik(void *pvParameters){
  for(;;){
    read_altitude();
    vTaskDelay(pdMS_TO_TICKS(Ultrasonik_time));
  }
};

void Print_task(void *pvParameters){
  for(;;){
      printUSB();
      vTaskDelay(pdMS_TO_TICKS(PRINT_time));
  }
};

void radio(void *pvParameters){
  for(;;){
      remote_loop();
      vTaskDelay(pdMS_TO_TICKS(RADIO_time));
  }
};

void controlThd(void *pvParameters){
  for(;;){
      Transition_sequence_manual();
      vTaskDelay(pdMS_TO_TICKS(CONTROL_time));
  }
};



void setup() {
  Wire.begin();
  Serial.begin(9600);
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();
  previousTime = millis(); 
  remote_setup();
  ultrasonic_setup();
  init_actuator();

  xTaskCreate(updateIMU, "IMU", 2048, NULL, 3, &Task_IMU);
  // xTaskCreate(radio, "Radio", 4096, NULL, 2, &Task_Radio); // Diprioritaskan di level 2
  // xTaskCreate(updateUltrasonik, "Ultrasonik", 2048, NULL, 3, &Task_Ultrasonik);
  xTaskCreate(Print_task, "Print", 4096, NULL, 3, &Task_Print);
  // xTaskCreate(controlThd, "Control", 4096, NULL, 3, NULL);
  vTaskStartScheduler(); // Start the scheduler
}

void loop() {
  // Kosong, semua dikendalikan oleh task
}
