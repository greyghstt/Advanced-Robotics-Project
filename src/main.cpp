#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <Kalman.h>
#include <Radio.h>
#include <Ultrasonik.h>
#include <Actuator.h>
#include <akuisisi.h>
#include <Transisition.h>
#include <Gcs_config.h>
#if ENABLE_WIFI_HTTP_TELEMETRY
#include <Telemetry.h>
#endif
#if ENABLE_BT_GCS
#include <BluetoothTelemetry.h>
#endif
#if ENABLE_UDP_GCS
#include <UdpTelemetry.h>
#endif
#include <utility/imumaths.h>

#define IMU_time 5
#define Ultrasonik_time 10
#define PRINT_time 50
#define CONTROL_time 5
#define RADIO_time 2 //10

TaskHandle_t Task_IMU;
TaskHandle_t Task_Ultrasonik;
TaskHandle_t Task_Print;
TaskHandle_t Task_Radio; // Tambahkan handle untuk task radio

void printUSB() {
  Serial.print("radio_ok:"); Serial.print(radio_frame_valid);
  Serial.print(" failsafe:"); Serial.print(radio_failsafe || signal_lost);
  Serial.print(" arm:"); Serial.print(arming);
  Serial.print(" mode:"); Serial.print(mode_now);
  Serial.print(" act:"); Serial.print(actuator_ready);
  Serial.print(" esc_test:"); Serial.print(ESC_DIRECT_THROTTLE_TEST);
  Serial.print(" pwm R/P/T/Y/A:");
  Serial.print(ch_roll); Serial.print("/");
  Serial.print(ch_pitch); Serial.print("/");
  Serial.print(ch_throttle); Serial.print("/");
  Serial.print(ch_yaw); Serial.print("/");
  Serial.print(arm);
  Serial.print(" motor:");
  Serial.print((int)m1_pwm); Serial.print("/");
  Serial.print((int)m2_pwm); Serial.print("/");
  Serial.print((int)m3_pwm); Serial.print("/");
  Serial.print((int)m4_pwm);
  Serial.print(" ctrl:");
  Serial.print(control1); Serial.print("/");
  Serial.print(control2); Serial.print("/");
  Serial.print(control3); Serial.print("/");
  Serial.print(control4);
  Serial.print(" omega:");
  Serial.print(omega2[0], 1); Serial.print("/");
  Serial.print(omega2[1], 1); Serial.print("/");
  Serial.print(omega2[2], 1); Serial.print("/");
  Serial.print(omega2[3], 1);
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
  Serial.begin(115200);
  // Wire.beginTransmission(MPU);
  // Wire.write(0x6B);
  // Wire.write(0);
  // Wire.endTransmission(); krn pake bno
  if(!bno.begin()) {
    Serial.println("BNO055 not detected");
    while(1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  // bno.setMode(OPERATION_MODE_NDOF); // Mode sensor fusion

  previousTime = millis(); 
  remote_setup();
  ultrasonic_setup();
  init_actuator();

#if ENABLE_WIFI_HTTP_TELEMETRY
  telemetry_setup();
#endif

#if ENABLE_BT_GCS
  bt_gcs_setup();
#endif

#if ENABLE_UDP_GCS
  udp_gcs_setup();
#endif

  xTaskCreate(updateIMU, "IMU", 2048, NULL, 3, &Task_IMU);
  xTaskCreate(radio, "Radio", 4096, NULL, 2, &Task_Radio); // Diprioritaskan di level 2
  //xTaskCreate(updateUltrasonik, "Ultrasonik", 2048, NULL, 3, &Task_Ultrasonik);
  // xTaskCreate(Print_task, "Print", 4096, NULL, 3, &Task_Print);
  xTaskCreate(controlThd, "Control", 4096, NULL, 3, NULL);

#if ENABLE_WIFI_HTTP_TELEMETRY
  xTaskCreate(telemetryTask, "Telemetry", 4096, NULL, 1, NULL);
#endif

#if ENABLE_BT_GCS
  xTaskCreate(btGcsTask, "BTGCS", 4096, NULL, 1, NULL);
#endif

#if ENABLE_UDP_GCS
  xTaskCreate(udpGcsTask, "UDPGCS", 4096, NULL, 1, NULL);
#endif

  vTaskStartScheduler(); // Start the scheduler
}

void loop() {
  // Kosong, semua dikendalikan oleh task
}
