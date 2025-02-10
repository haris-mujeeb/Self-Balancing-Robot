#include "pins.hpp"
#include "mpu6050_base.hpp"

mpu6050_base mpu;

void setup() {
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);
  Serial.begin(9600);
  Wire.begin();
  mpu.init();
}

void loop() {
  mpu.calculate();
  mpu.angle_pitch_gyro;
  mpu.testCompFilt(0.9);
  // mpu.testCompFilt(0.95);
  // mpu.testCompFilt(0.99);
  // delay(50);
}