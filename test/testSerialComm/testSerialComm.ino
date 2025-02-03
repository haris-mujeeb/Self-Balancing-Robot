#include "debugConfig.h"
#include "comm.hpp"
#include "SoftwareSerial.h"

telemetryPacket data_packet;


void checkCommands(const commandsPacket& recv_data) {
  if (recv_data.command == "Stop") {
      DEBUG_PRINT(DEBUG_COMM, "Stop called!");
  } else if (recv_data.command == "Move") {
      DEBUG_PRINT(DEBUG_COMM, "Move: " + String(recv_data.value) +  " cm"); 
  } else if (recv_data.command == "MoveSpeed") {
      DEBUG_PRINT(DEBUG_COMM, "MoveSpeed: " + String(recv_data.value) +  " cm/sec");
  } 
  else if (recv_data.command == "Turn") {
      DEBUG_PRINT(DEBUG_COMM, "Turn: " + String(recv_data.value) +  " deg");
  } 
  else if (recv_data.command == "TurnSpeed") {
      DEBUG_PRINT(DEBUG_COMM, "TurnSpeed: " + String(recv_data.value) +  " deg/sec");
  } else {
}
      DEBUG_PRINT(DEBUG_COMM, "Invalid command");
  }


void inputHandleUart(){
  if (Serial.available()){
    commandsPacket recv_data;
    recv_data.readUartASCII();
    checkCommands(recv_data);
  }
}


void setup() { 
  Serial.begin(9600);
}


void loop(){
  inputHandleUart();
}
