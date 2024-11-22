#include "debugConfig.h"
#include "motion_controller.hpp"

motion_controller motion;

void inputHandle();

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
  inputHandle();
}


void inputHandle(){
  if(Serial.available()){
    char input = Serial.read();
    if (input != '\0'){
      switch (input)
      {
      case 'w':
        motion.moveForward(40);  // 40 or 80
        break;
      case 's':
        motion.moveBack(40);  // 40 or 80
        break;
      case 'a':
        motion.turnLeft(20);  // 50
        break;
      case 'd':
          motion.turnRight(20);  // 50
        break;
      case 'f':
          motion.stop();  // 50
        break;
      default:
        break;
      }
    }
  }
}