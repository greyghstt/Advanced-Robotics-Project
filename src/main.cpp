#include <Arduino.h>
#include <Wire.h>

#include <Radio.h>
#include <Ultrasonic.h>
#include <Actuator.h>
#include <ImuSensor.h>
#include <FlightSafety.h>
#include <GcsConfig.h>

#if ENABLE_UDP_GCS
#include <UdpTelemetry.h>
#endif

static const uint16_t IMU_PERIOD_MS = 5;
static const uint16_t CONTROL_PERIOD_MS = 5;
static const uint16_t RADIO_PERIOD_MS = 2;
static const uint16_t DEBUG_PERIOD_MS = 100;

static const BaseType_t CONTROL_CORE = 1;
static const BaseType_t WIFI_CORE = 0;

TaskHandle_t task_imu = nullptr;
TaskHandle_t task_radio = nullptr;
TaskHandle_t task_control = nullptr;
TaskHandle_t task_udp_gcs = nullptr;
TaskHandle_t task_debug = nullptr;

void printUSB() {
    Serial.print("radio_ok:"); Serial.print(radio_frame_valid);
    Serial.print(" failsafe:"); Serial.print(radio_failsafe || signal_lost);
    Serial.print(" arm:"); Serial.print(arming);
    Serial.print(" safety:"); Serial.print(flight_safety_reason);
    Serial.print(" imu:"); Serial.print(imu_ready);
    Serial.print(" cal:"); Serial.print(imu_cal_sys); Serial.print("/");
    Serial.print(imu_cal_gyro); Serial.print("/");
    Serial.print(imu_cal_accel); Serial.print("/");
    Serial.print(imu_cal_mag);
    Serial.print(" mode:"); Serial.print(mode_now);
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
    Serial.println();
}

void taskImu(void *pvParameters) {
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        imu_update();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(IMU_PERIOD_MS));
    }
}

void taskRadio(void *pvParameters) {
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        remote_loop();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(RADIO_PERIOD_MS));
    }
}

void taskControl(void *pvParameters) {
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        flight_safety_update();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void taskDebug(void *pvParameters) {
    (void)pvParameters;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        printUSB();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(DEBUG_PERIOD_MS));
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    if (!imu_setup()) {
        Serial.println("BNO055 not detected");
        while (true) {
            delay(1000);
        }
    }

    remote_setup();
    ultrasonic_setup();
    init_actuator();

#if ENABLE_UDP_GCS
    udp_gcs_setup();
#endif

    xTaskCreatePinnedToCore(taskRadio, "Radio", 4096, nullptr, 5, &task_radio, CONTROL_CORE);
    xTaskCreatePinnedToCore(taskImu, "IMU", 4096, nullptr, 4, &task_imu, CONTROL_CORE);
    xTaskCreatePinnedToCore(taskControl, "Control", 4096, nullptr, 4, &task_control, CONTROL_CORE);

#if ENABLE_UDP_GCS
    xTaskCreatePinnedToCore(udpGcsTask, "UDPGCS", 4096, nullptr, 1, &task_udp_gcs, WIFI_CORE);
#endif

    // Keep disabled by default; UDP dashboard already logs telemetry.
    // xTaskCreatePinnedToCore(taskDebug, "Debug", 4096, nullptr, 1, &task_debug, WIFI_CORE);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
