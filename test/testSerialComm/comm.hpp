#include <stdint.h>
#include "Arduino.h"
#include <Wire.h>


bool isNumeric(const String& str);


struct telemetryPacket {
  float yaw = 0.0;
  float distance = 0.0;
  void sendI2CBytes(uint8_t address) const;
  void sendI2CASCII(uint8_t address) const;
  void sendUartBytes() const;
  void sendUartASCII() const;
  void readI2CBytes(uint8_t address);
  void readI2CASCII(uint8_t address);
  void readUartBytes();
  void readUartASCII();
};


void telemetryPacket::sendI2CBytes(uint8_t address) const {
  Wire.beginTransmission(address);
  Wire.write(reinterpret_cast<const uint8_t*>(&yaw), sizeof(yaw));
  Wire.write(reinterpret_cast<const uint8_t*>(&distance), sizeof(distance));
  Wire.endTransmission();
}


void telemetryPacket::sendI2CASCII(uint8_t address) const{
  Wire.beginTransmission(address);
  char buffer[50];
  sprintf(buffer, "yaw,%.6f,distance,%.6f\n", yaw, distance);
  Wire.print(buffer);
  Wire.endTransmission();
}


// Function to send packet as raw bytes (binary)
void telemetryPacket::sendUartBytes() const {
  Serial.write(reinterpret_cast<const uint8_t*>(&yaw), sizeof(yaw));  // Send yaw as 4 bytes
  Serial.write(reinterpret_cast<const uint8_t*>(&distance), sizeof(distance));  // Send distance as 4 bytes
}


// Function to send packet as ASCII (human-readable)
void telemetryPacket::sendUartASCII() const {
  char buffer[50];
  sprintf(buffer, "yaw,%.6f,distance,%.6f\n", yaw, distance);
  Serial.println(buffer);
}

void telemetryPacket::readI2CBytes(uint8_t address) {
  Wire.requestFrom(address, (uint8_t)(sizeof(yaw) + sizeof(distance)));
  if (Wire.available() < sizeof(yaw) + sizeof(distance)) {
    DEBUG_PRINT(DEBUG_COMM, "Insufficient I2C data received.");
    return;
  }
  Wire.readBytes(reinterpret_cast<uint8_t*>(&yaw), sizeof(yaw));
  Wire.readBytes(reinterpret_cast<uint8_t*>(&distance), sizeof(distance));
  DEBUG_PRINT(DEBUG_COMM, "I2C Yaw: " + String(yaw));
  DEBUG_PRINT(DEBUG_COMM, "I2C Distance: " + String(distance));
}

void telemetryPacket::readI2CASCII(uint8_t address) {
  Wire.requestFrom(address, (uint8_t)50);
  String input = "";
  while (Wire.available()) {
    input += (char)Wire.read();
  }
  input.trim();
  if (input.length() == 0) {
    DEBUG_PRINT(DEBUG_COMM, "No input received over I2C.");
    return;
  }
  int yawIndex = input.indexOf("yaw,");
  int distIndex = input.indexOf(",distance,");
  if (yawIndex != -1 && distIndex != -1) {
    yaw = input.substring(yawIndex + 4, distIndex).toFloat();
    distance = input.substring(distIndex + 10).toFloat();
  }
  DEBUG_PRINT(DEBUG_COMM, "I2C Yaw: " + String(yaw));
  DEBUG_PRINT(DEBUG_COMM, "I2C Distance: " + String(distance));
}

void telemetryPacket::readUartBytes() {
  if (Serial.available() < sizeof(yaw) + sizeof(distance)) {
    DEBUG_PRINT(DEBUG_COMM, "Insufficient UART data received.");
    return;
  }
  Serial.readBytes(reinterpret_cast<uint8_t*>(&yaw), sizeof(yaw));
  Serial.readBytes(reinterpret_cast<uint8_t*>(&distance), sizeof(distance));
  DEBUG_PRINT(DEBUG_COMM, "UART Yaw: " + String(yaw));
  DEBUG_PRINT(DEBUG_COMM, "UART Distance: " + String(distance));
}

void telemetryPacket::readUartASCII() {
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) {
    DEBUG_PRINT(DEBUG_COMM, "No UART input received.");
    return;
  }
  int yawIndex = input.indexOf("yaw,");
  int distIndex = input.indexOf(",distance,");
  if (yawIndex != -1 && distIndex != -1) {
    yaw = input.substring(yawIndex + 4, distIndex).toFloat();
    distance = input.substring(distIndex + 10).toFloat();
  }
  DEBUG_PRINT(DEBUG_COMM, "UART Yaw: " + String(yaw));
  DEBUG_PRINT(DEBUG_COMM, "UART Distance: " + String(distance));
}


struct commandsPacket {
  String command;
  int value = 0.0;
  void sendI2CBytes(uint8_t address) const;
  void sendI2CASCII(uint8_t address) const;
  void sendUartBytes() const;
  void sendUartASCII() const;
  void readI2CBytes(uint8_t address);
  void readI2CASCII(uint8_t address);
  void readUartBytes();
  void readUartASCII();
};


void commandsPacket::sendI2CBytes(uint8_t address) const {
  Wire.beginTransmission(address);
  Wire.write(command.charAt(0));
  Wire.write(reinterpret_cast<const uint8_t*>(&value), sizeof(value));
  Wire.endTransmission();
}


void commandsPacket::sendI2CASCII(uint8_t address) const {
  Wire.beginTransmission(address);
  String packet = command + "," + String(value) + "\n";
  Wire.print(packet);
  Wire.endTransmission();
}


void commandsPacket::sendUartBytes() const {
  Serial.write(command.charAt(0));
  Serial.write(reinterpret_cast<const uint8_t*>(&value), sizeof(value));
}


void commandsPacket::sendUartASCII() const {
  Serial.println(command + "," + String(value));
}


void commandsPacket::readI2CBytes(uint8_t address){
  Wire.requestFrom(address, (uint8_t)5);

  if (Wire.available() < 5) {
    DEBUG_PRINT(DEBUG_COMM, "Insufficient I2C data received.");
    return;
  }

  char cmdByte = Wire.read();
  command = String(cmdByte);

  uint8_t valueBytes[4];
  for (int i = 0; i < 4; i++) {
    valueBytes[i] = Wire.read();
  }

  float* valuePtr = reinterpret_cast<float*>(valueBytes);
  value = *valuePtr;

  DEBUG_PRINT(DEBUG_COMM, "I2C Command: " + command);
  DEBUG_PRINT(DEBUG_COMM, "I2C Value: " + String(value));
}


void commandsPacket::readI2CASCII(uint8_t address){
  Wire.requestFrom(address, (uint8_t)5);

  if (Wire.available() < 5) {
    DEBUG_PRINT(DEBUG_COMM, "Insufficient I2C data received.");
    return;
  }

  char cmdByte = Wire.read();
  command = String(cmdByte);

  uint8_t valueBytes[4];
  for (int i = 0; i < 4; i++) {
    valueBytes[i] = Wire.read();
  }

  float* valuePtr = reinterpret_cast<float*>(valueBytes);
  value = *valuePtr;

  DEBUG_PRINT(DEBUG_COMM, "I2C Command: " + command);
  DEBUG_PRINT(DEBUG_COMM, "I2C Value: " + String(value));
}


void commandsPacket::readUartBytes(){
  // Read the first byte to get the command
  if (Serial.available() < 2) {
    DEBUG_PRINT(DEBUG_COMM, "Insufficient data received.");
    return;
  }

  char cmdByte = Serial.read();
  command = String(cmdByte);
  
  uint8_t valueBytes[4];
  for (int i = 0; i < 4 && Serial.available(); i++) {
    valueBytes[i] = Serial.read();
  }

  float* valuePtr = reinterpret_cast<float*>(valueBytes);
  value = *valuePtr;

  DEBUG_PRINT(DEBUG_COMM, "Command: " + command);
  DEBUG_PRINT(DEBUG_COMM, "Value: " + String(value));
}


void commandsPacket::readUartASCII() {
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (!input.length()) {
    DEBUG_PRINT(DEBUG_COMM, "No input received.");
    command = "";
    return;
  }

  int commaIndex = input.indexOf(',');
  if (commaIndex == -1) {
    DEBUG_PRINT(DEBUG_COMM, "Comma not found in input.");
    command = "";
    return;
  }

  command = input.substring(0, commaIndex);
  command.trim();
  String valueString = input.substring(commaIndex + 1);
  valueString.trim();

  if (!command.length()) {
    DEBUG_PRINT(DEBUG_COMM, "Empty command.");
    command = "";
    return;
  }

  if (isNumeric(valueString)) {
    value = valueString.toFloat();
  } else {
    DEBUG_PRINT(DEBUG_COMM, "Invalid value.");
    value = 0.0;
  }

  DEBUG_PRINT(DEBUG_COMM, command);
  DEBUG_PRINT(DEBUG_COMM, value);
}


bool isNumeric(const String& str){
  if (!str.length()) return false;
  bool decimalPointSeen = false;
  for (unsigned int i = 0; i < str.length(); i++) {
    char c = str.charAt(i);
    if (!isDigit(c)) {
      if (c == '.' && !decimalPointSeen) {
        decimalPointSeen = true;
      } else if (i == 0 && c == '-') {
        continue;
      } else {
        return false;
      }
    }
  }
  return true;
}
