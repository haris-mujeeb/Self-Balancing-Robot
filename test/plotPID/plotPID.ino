#include "debugConfig.h"
#include "motion_controller.hpp"

motion_controller motion;

void setup() {
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);
  Serial.begin(250000);
  Wire.begin();
  motion.init();
}

void loop() {

}