#include "debugConfigTest.h"
#include "motion_controller.hpp"

motionController motion;

void testYawStep();

void setup() {
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  Wire.begin();
  motion.run();
}

void loop() {
  // testYawStep();
  // testPositionStep();
}

void testYawStep() {
  motion.rotate(-45);
  delay(6000);
  motion.rotate(45);
  delay(6000);
}

void testPositionStep() {
  motion.move(-30);
  delay(6000);
  motion.move(30);
  delay(6000);
}