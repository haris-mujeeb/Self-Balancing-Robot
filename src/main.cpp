#include "debugConfig.h"
#include "motionController.hpp"
#include "comm.hpp"
#include "SoftwareSerial.h"
#include "ultrasonic.hpp"

// void inputHandleOLD();
void inputHandle();
void test_moving();
void test_turning();

motionController robot;
telemetryPacket data_packet;

void setup() { 
  Serial.begin(9600); // 250000  
  InitializeIRAndUltrasonic();
  wdt_disable();
  robot.run();
  DEBUG_PRINT(DEBUG_MODE, "Robot initiated.");
}

void loop() {
  static unsigned long updateTime = 0;
  
  CheckIRObstacle();
  
  if(millis() - updateTime > 1000) {
    updateTime = millis();

    StartUltrasonicMeasurement();
    
    robot.getRobotStateData(data_packet.distance, data_packet.yaw);
    // data_packet.sendPacketASCII();
  }
  
  if(currentRobotState != STARTING) {
    if(irLeftIsObstacle && irRightIsObstacle && usonicDistanceValue < 20.0) {
      robot.stop();
      while (usonicDistanceValue > 20)  {
        robot.moveCentimeters(robot_position - 1);
      }
    }
  }
  
  inputHandle();
}

void inputHandle(){
  if (Serial.available()){
    telemetryCommands recv_data;
    recv_data.readPacketASCII();
    switch (recv_data.command) {
      case 'M':
        robot.moveCentimeters((float)recv_data.value);
        DEBUG_PRINT(DEBUG_COMM, "Move: " + String(recv_data.value) +  " cm");
        break;
      case 'T':
        robot.turnDegrees((float)recv_data.value);
        DEBUG_PRINT(DEBUG_COMM, "Turn: " + String(recv_data.value) +  " deg");
        break;
      case '\0':
        DEBUG_PRINT(DEBUG_COMM, "No Command.");
        break;
      default:
        DEBUG_PRINT(DEBUG_COMM, "Invalid command");
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
      robot.moveCentimeters(30);
      break;
    case 1:
      robot.moveCentimeters(0);
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
      robot.turnDegrees(45);
      break;
    case 1:
      robot.turnDegrees(-45);
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