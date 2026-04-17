#ifndef ULTRASONIK_H
#define ULTRASONIK_H
#include <Arduino.h>

#define TRIG_PIN 12
#define ECHO_PIN 13

float altitude = 0.0f;

void ultrasonic_setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);
}


float read_altitude() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  float distance = (duration * 0.034) / 2; // cm

  if (distance < 2 || distance > 400) {
    return -1; // Invalid distance
  }
  altitude = distance;
  return distance;
}

#endif // ULTRASONIK_H
