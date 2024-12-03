#include "debugConfig.h"
#include "motionController.hpp"
#include "comm.hpp"
#include "SoftwareSerial.h"

// void inputHandleOLD();
void inputHandle();
void test_moving();
void test_turning();

motionController motion_controller;
telemetryPacket data_packet;

void setup() { 
  Serial.begin(9600); // 250000
  while (!Serial) {    // Wait for Serial to be ready (optional for some platforms)
    delay(10);
  }
  
  // Serial.println("AT+UART=9600,0,0"); // Set BT baud rate to 9600
  // delay(1000);
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);
  
  wdt_disable();
  motion_controller.run();
  DEBUG_PRINT(DEBUG_MODE, "Robot initiated.");
}

void loop() {
  motion_controller.getRobotStateData(data_packet.distance, data_packet.yaw);
  static unsigned long updateTime = 0;
  if(millis() - updateTime > 1000) {
    data_packet.sendPacketASCII();
    updateTime = millis();
  }
  inputHandle();

  // test_moving();
  // test_turning();
}

void inputHandle(){
  if (Serial.available()){
    telemetryCommands recv_data;
    recv_data.readPacketASCII();
    switch (recv_data.command) {
      case 'M':
        motion_controller.moveCentimeters((float)recv_data.value);
        DEBUG_PRINT(DEBUG_COMM, "Move: " + String(recv_data.value) +  " cm");
        break;
      case 'T':
        motion_controller.turnDegrees((float)recv_data.value);
        DEBUG_PRINT(DEBUG_COMM, "Turn: " + String(recv_data.value) +  " deg");
        break;
      case '\0':
        DEBUG_PRINT(DEBUG_COMM, "No Command.");
        break;
      default:
        ERROR_PRINT("Invalid command");
        break;
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
      motion_controller.moveCentimeters(30);
      break;
    case 1:
      motion_controller.moveCentimeters(0);
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
      motion_controller.turnDegrees(45);
      break;
    case 1:
      motion_controller.turnDegrees(-45);
      break;
    default:
      break;
    }
}


// void inputHandleOLD(){
//     if(Serial.available()){
//     char input = '\0';
//     input =  Serial.read();
//     if (input != '\0') {
//       switch (input) {
//       case 'q':
//         // motionController.moveForward(20);  // 40 or 80
//         motionController.moveCentimeters(30);
//         break;
//       case 'w':
//         // motionController.moveForward(20);  // 40 or 80
//         motionController.moveCentimeters(20);
//         break;
//       case 'e':
//         // motionController.moveForward(20);  // 40 or 80
//         motionController.moveCentimeters(10);
//         break;
//       case 's':
//         // motionController.moveBack(60);  // 40 or 80
//         motionController.moveCentimeters(0);
//         break;
//       case 'a':
//         // motionController.turnLeft(50);  // 50
//         motionController.turnDegrees(45);
//         break;
//       case 'd':
//           // motionController.turnRight(50);  // 50
//         motionController.turnDegrees(-45);
//         break;
//       case 'f':
//           motionController.stop();  
//         break;
//       default:
//         break;
//       }
//     DEBUG_PRINT(true ,"input called!");
//     }
//   }
// }