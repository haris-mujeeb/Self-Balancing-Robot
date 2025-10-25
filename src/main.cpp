#include "debugConfig.h"
#include "motionController.hpp"
#include "comm.hpp"
#include "SoftwareSerial.h"
#include "ultrasonic.hpp"

void updateCMD();
void i2cRecvHandle(int numBytes);
void i2cRequestEvent();
void checkUltraSonicObstacle();
void serialInputHandle();

motionController robot;
telemetryPacket tele_data;
commandPacket cmd;
const uint8_t USONIC_HARD_MIN_DIST = 15;
const uint8_t USONIC_SOFT_MIN_DIST = 80;
const uint8_t UPDATE_DATA_MS = 50;
bool obstacleDetected = false;
bool cannotMoveForward = false;
bool robotStopped = false;

void setup() { 
  while(!digitalRead(KEY_MODE)) {}
  Serial.begin(115200); // for Bluetooth 
  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(i2cRecvHandle);
  Wire.onRequest(i2cRequestEvent);
  InitializeIRAndUltrasonic();
  wdt_disable();
  robot.run();
  DEBUG_PRINT(DEBUG_MODE, "Robot initiated.");
}

void loop() {
  static unsigned long updateTime = 0;
  CheckIRObstacle();

  if(millis() - updateTime > UPDATE_DATA_MS) {
    updateTime = millis();

    // Get Ultrasonic measurements
    StartUltrasonicMeasurement();
    // Check for obstacle using Ultrasonic 
    checkUltraSonicObstacle();

    // Get the telemetry data packet
    robot.getRobotStateData(tele_data);

    // Update the telemetry data packet
    tele_data.ultrasonicDistanceCm = usonicDistanceValue;
    tele_data.leftIR_Detected = irLeftIsObstacle;
    tele_data.rightIR_Detected = irRightIsObstacle;
    tele_data.sendUartASCII(Serial);
  }



  // if(usonicDistanceValue > 15) {obstacleDetected = false;}

  serialInputHandle();
  updateCMD();
}


void serialInputHandle(){
  if (Serial.available()){
    cmd.readUartASCII(Serial);
    // updateCMD();
  }
}

void i2cRecvHandle(int numBytes){
  cmd.readI2CBytes(numBytes);
  // cmd.readI2CASCII(numBytes);
  // updateCMD();
}


void i2cRequestEvent(){
  tele_data.sendI2CBytes();
  // tele_data.sendI2CASCII(Serial);
}


void checkUltraSonicObstacle() {
  if(usonicDistanceValue < USONIC_SOFT_MIN_DIST) {
    if(cmd.command == Move) {
      float minimum_safe_distance = tele_data.robotDistanceCm + usonicDistanceValue - USONIC_HARD_MIN_DIST;
      cmd.commandValue = constrain(cmd.commandValue, cmd.commandValue, minimum_safe_distance);
      cmd.commandSpeed = 20;
      // robot.move((float)cmd.commandValue, (float)cmd.commandSpeed);
    }

    obstacleDetected = true;

    if (usonicDistanceValue < USONIC_HARD_MIN_DIST) {
        cannotMoveForward = true;
      } else {
        cannotMoveForward = false;
      }

  } else {
    obstacleDetected = false;
  }
}


void updateCMD() {
  switch (cmd.command) {
    case Stop:
      if(robotStopped == false) {
        robot.stop();
        robotStopped = true;
      }
    DEBUG_PRINT(DEBUG_COMM, "Stop called!");
    break;
    case Move:
      robotStopped = false;

      if (cannotMoveForward) {
        cmd.commandValue = constrain(cmd.commandValue, cmd.commandValue, tele_data.robotDistanceCm);
        cmd.commandSpeed = 30;        
      }
      robot.move((float)cmd.commandValue, (float)cmd.commandSpeed);
      
      if (abs(cmd.commandValue - tele_data.robotDistanceCm) < 5) {
        cmd.command = Stop;
        cmd.commandValue = 0;
        cmd.commandSpeed = 0;
      }

      DEBUG_PRINT(DEBUG_COMM, "Move: " + String(cmd.commandValue) +  " cm");    
      DEBUG_PRINT(DEBUG_COMM, "Move Speed: " + String(cmd.commandSpeed) +  " %");    
      break;
    case Rotate:
      robotStopped = false;
      robot.rotate((float)cmd.commandValue, (float)cmd.commandSpeed);

      if (abs(cmd.commandValue - tele_data.robotDistanceCm) < 1) {
        cmd.command = Stop;
        cmd.commandValue = 0;
        cmd.commandSpeed = 0;
      }

      DEBUG_PRINT(DEBUG_COMM, "Turn: " + String(cmd.commandValue) +  " deg");
      DEBUG_PRINT(DEBUG_COMM, "Turn Speed: " + String(cmd.commandSpeed) +  " %");
      break;
    default:
        DEBUG_PRINT(DEBUG_COMM, "Invalid command");
        break;
    }
}