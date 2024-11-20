#include "debugConfig.h"
#include "motion_controller.hpp"

motion_controller motion;

void setup() {
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);
  Serial.begin(250000);

  // DEBUG_PRINT(DEBUG_MODE, "[Debug] Press the buttton to start!");
  // while(digitalRead(KEY_MODE));  // Stop execution until Push button is pressed.
  // DEBUG_PRINT(DEBUG_MODE, "[Debug] Starting the loop...");

  motion.init();
  DEBUG_PRINT(DEBUG_MODE, "[Debug] Robot initiated.");
}

void loop() {

}