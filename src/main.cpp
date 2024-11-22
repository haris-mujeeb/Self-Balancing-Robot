#include "debugConfig.h"
#include "motion_controller.hpp"

motion_controller motionController;

void inputHandle();

void setup() {
  Serial.begin(250000);
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);
  wdt_disable();
  motionController.run();
  DEBUG_PRINT(DEBUG_MODE, "Robot initiated.");
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
        motionController.moveForward(60);  // 40 or 80
        break;
      case 's':
        motionController.moveBack(60);  // 40 or 80
        break;
      case 'a':
        motionController.turnLeft(20);  // 50
        break;
      case 'd':
          motionController.turnRight(20);  // 50
        break;
      case 'f':
          motionController.stop();  // 50
        break;
      default:
        break;
      }
    }
  }
}