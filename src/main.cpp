#include "debugConfig.h"
#include "motionController.hpp"
#include "comm.hpp"
#include "SoftwareSerial.h"
#include "ultrasonic.hpp"

void updateCMD();
void i2cRecvHandle(int numBytes);
void i2cRequestEvent();
void  serialInputHandle();

motionController robot;
telemetryPacket tele_data;
commandPacket cmd;
const uint8_t USONIC_HARD_MIN_DIST = 30.0;
const uint8_t UPDATE_DATA_MS = 50;
bool obstacleDetected = false;

void setup() { 
  while(!digitalRead(KEY_MODE)) {}
  Serial.begin(9600); // for Bluetooth 
  // Serial.begin(115200);   // for Terra
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
    StartUltrasonicMeasurement();
    robot.getRobotStateData(tele_data);
    tele_data.ultrasonicDistanceCm = usonicDistanceValue;
    // Test
    tele_data.sendUartASCII();
  }

  if (irLeftIsObstacle && irRightIsObstacle && (usonicDistanceValue < 20)) {
    if (!obstacleDetected) {
      robot.move((float)(tele_data.robotDistanceCm - 10), (float)cmd.commandSpeed);
      obstacleDetected = true;   
    } 
  }

  if(usonicDistanceValue > 15) {
      obstacleDetected = false;   
  }

  serialInputHandle();

}


void serialInputHandle(){
  if (Serial.available()){
    cmd.readUartASCII();
    updateCMD();
  }
}

void i2cRecvHandle(int numBytes){
  cmd.readI2CBytes(numBytes);
  updateCMD();
}


void updateCMD() {
  switch (cmd.command) {
    case Stop:
      robot.stop();
      DEBUG_PRINT(DEBUG_COMM, "Stop called!");
      break;
    case Move:
      if (obstacleDetected){
        cmd.commandValue = constrain(cmd.commandValue, cmd.commandValue, tele_data.robotDistanceCm);
      }
      robot.move((float)cmd.commandValue, (float)cmd.commandSpeed);

      DEBUG_PRINT(DEBUG_COMM, "Move: " + String(cmd.commandValue) +  " cm");    
      DEBUG_PRINT(DEBUG_COMM, "Move: " + String(cmd.commandSpeed) +  " cm");    
      break;
    case Rotate:
      robot.rotate((float)cmd.commandValue, (float)cmd.commandSpeed);
      DEBUG_PRINT(DEBUG_COMM, "Turn: " + String(cmd.commandValue) +  " deg");
      DEBUG_PRINT(DEBUG_COMM, "Turn Speed: " + String(cmd.commandSpeed) +  " deg/sec");
      break;
    default:
        DEBUG_PRINT(DEBUG_COMM, "Invalid command");
        break;
    }
}


void i2cRequestEvent(){
  // tele_data.readI2CBytes();
  tele_data.sendUartASCII();
}
