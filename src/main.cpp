#include "debugConfig.h"
#include "motionController.hpp"
#include "comm.hpp"
#include "SoftwareSerial.h"
#include "ultrasonic.hpp"

void inputHandle();

motionController robot;
telemetryPacket data_packet;

void setup() { 
  Serial.begin(9600); // for Bluetooth 
  // Serial.begin(115200);   // for Terra
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
    recv_data.readUartASCII();

    if (recv_data.command == "Stop") {
        robot.stop();
        DEBUG_PRINT(DEBUG_COMM, "Stop called!");
    } else if (recv_data.command == "Move") {
        robot.moveCentimeters((float)recv_data.value);
        DEBUG_PRINT(DEBUG_COMM, "Move: " + String(recv_data.value) +  " cm");
    } 
    else if (recv_data.command == "Turn") {
        robot.turnDegrees((float)recv_data.value);   
        DEBUG_PRINT(DEBUG_COMM, "Turn: " + String(recv_data.value) +  " deg");
    } 
    else if (recv_data.command == "TurnSpeed") {
        robot.turnSpeed((float)recv_data.value);   
        DEBUG_PRINT(DEBUG_COMM, "Turn Speed: " + String(recv_data.value) +  " deg/sec");
    } else {
        DEBUG_PRINT(DEBUG_COMM, "Invalid command");
    }
  }
}