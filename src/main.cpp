#include "debugConfig.h"
#include "motion_controller.hpp"


void inputHandle();
void test_moving();
void test_turning();

motion_controller motionController;

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
  // test_moving();
  test_turning();
}


void inputHandle(){
  if(Serial.available()){
    char input = '\0';
    input = Serial.read();
    if (input != '\0'){
      switch (input)
      {
      case 'q':
        // motionController.moveForward(20);  // 40 or 80
        motionController.moveCentimeters(30);
        break;
      case 'w':
        // motionController.moveForward(20);  // 40 or 80
        motionController.moveCentimeters(20);
        break;
      case 'e':
        // motionController.moveForward(20);  // 40 or 80
        motionController.moveCentimeters(10);
        break;
      case 's':
        // motionController.moveBack(60);  // 40 or 80
        motionController.moveCentimeters(0);
        break;
      case 'a':
        // motionController.turnLeft(50);  // 50
        motionController.turnDegrees(45);
        break;
      case 'd':
          // motionController.turnRight(50);  // 50
        motionController.turnDegrees(-45);
        break;
      case 'f':
          motionController.stop();  
        break;
      default:
        break;
      }
    DEBUG_PRINT(true ,"input called!");
    }
  }
}

void test_moving() {
    static unsigned long testTimeLastUpdate = 0;
    static uint8_t state = 0;
    if (millis() - testTimeLastUpdate > 3000) {
      state++;
      state %= 2;
      testTimeLastUpdate = millis();
    }
    switch (state) {
    case 0:
      motionController.moveCentimeters(30);
      break;
    case 1:
      motionController.moveCentimeters(0);
      break;
    default:
      break;
    }
}


void test_turning() {
    static unsigned long testTimeLastUpdate = 0;
    static uint8_t state = 0;
    if (millis() - testTimeLastUpdate > 5000) {
      state++;
      state %= 2;
      testTimeLastUpdate = millis();
    }
    switch (state) {
    case 0:
      motionController.turnDegrees(45);
      break;
    case 1:
      motionController.turnDegrees(-45);
      break;
    default:
      break;
    }
}