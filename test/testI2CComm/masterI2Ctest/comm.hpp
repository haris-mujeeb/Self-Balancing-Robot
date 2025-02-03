#include "avr/pgmspace.h"
#include <stdint.h>
#include "Arduino.h"
#include <Wire.h>

#define BUFFER_SIZE 32

bool isNumeric(const String& str);

enum TumblerCmd : uint8_t {
  Stop,
  Move,
  Rotate,
  setSpeed,
  setRotSpeed,
  INVALID
};

TumblerCmd stringToCommand(const String& commandStr) {
  if (commandStr == "Stop") return Stop;
  if (commandStr == "Move") return Move;
  if (commandStr == "Rotate") return Rotate;
  if (commandStr == "setSpeed") return setSpeed;
  if (commandStr == "setRotSpeed") return setRotSpeed;
  return INVALID;  // Return an invalid command if none match
}

String commandToString(TumblerCmd cmd) {
  switch (cmd) {
    case Stop: return "Stop";
    case Move: return "Move";
    case Rotate: return "Rotate";
    case setSpeed: return "setSpeed";
    case setRotSpeed: return "setRotSpeed";
    default: return "Unknown";
  }
}

struct telemetryPacket {
  int16_t yaw_ = 0;
  long distance_ = 0;
  void sendI2CBytes() const;
  void sendI2CASCII() const;
  void sendUartBytes() const;
  void sendUartASCII() const;
  void readI2CBytes(uint8_t address);
  void readI2CASCII(uint8_t address);
  void readUartBytes();
  void readUartASCII();
};


void telemetryPacket::sendI2CBytes() const {
  Wire.write(reinterpret_cast<const uint8_t*>(&yaw_), sizeof(yaw_));
  Wire.write(reinterpret_cast<const uint8_t*>(&distance_), sizeof(distance_));
  DEBUG_PRINT(DEBUG_COMM, "Sent I2C: " + String(yaw_) + "," + String(distance_));
}


void telemetryPacket::sendI2CASCII() const {
  char buffer[BUFFER_SIZE];
  char yawStr[7];        // Buffer for yaw string (6 characters + 1 for null terminator)
  char distanceStr[12];  // Buffer for distance string (11 characters + 1 for null terminator)
  sprintf(yawStr, "%d", yaw_);
  sprintf(distanceStr, "%ld", distance_);

  snprintf(buffer, BUFFER_SIZE, "%s,%s", yawStr, distanceStr);
  Wire.write(buffer);
  DEBUG_PRINT(DEBUG_COMM, buffer);
}


// Function to send packet as raw bytes (binary)
void telemetryPacket::sendUartBytes() const {
  Serial.write(reinterpret_cast<const uint8_t*>(&yaw_), sizeof(yaw_));            // Send yaw_ as 4 bytes
  Serial.write(reinterpret_cast<const uint8_t*>(&distance_), sizeof(distance_));  // Send distance_ as 4 bytes
}


// Function to send packet as ASCII (human-readable)
void telemetryPacket::sendUartASCII() const {
  char buffer[BUFFER_SIZE];
  sprintf(buffer, "yaw_,%.6f,distance_,%.6f\n", yaw_, distance_);
  Serial.println(buffer);
}


void telemetryPacket::readI2CBytes(uint8_t address) {
  uint8_t bytesRequested = sizeof(yaw_) + sizeof(distance_);
  Wire.requestFrom(address, bytesRequested);

  if (Wire.available() < bytesRequested) {
    DEBUG_PRINT(DEBUG_COMM, "Insufficient I2C data received.");
    return;
  }
  Wire.readBytes(reinterpret_cast<uint8_t*>(&yaw_), sizeof(yaw_));
  Wire.readBytes(reinterpret_cast<uint8_t*>(&distance_), sizeof(distance_));
  DEBUG_PRINT(DEBUG_COMM, "Recv I2C : " + String(yaw_) + "," + String(distance_));
}


void telemetryPacket::readI2CASCII(uint8_t address) {
  char buffer[BUFFER_SIZE];
  Wire.requestFrom(address, BUFFER_SIZE - 1);

  if (!Wire.available()) {
    DEBUG_PRINT(DEBUG_COMM, "No response from I2C device.");
    return;
  } else if (Wire.available() < BUFFER_SIZE - 1) {
    DEBUG_PRINT(DEBUG_COMM, "Insufficient I2C data received.");
    return;
  }

  int index = 0;
  while (Wire.available() && index < BUFFER_SIZE - 1) {
    buffer[index++] = Wire.read();  // Read a byte
  }
  buffer[index] = '\0';  // Null-terminate the string

  // Parse the received data
  char* token = strtok(buffer, ",");
  if (token != NULL) {
    yaw_ = atoi(token);  // Convert the first token to int16_t
    token = strtok(NULL, ",");
    if (token != NULL) {
      distance_ = atol(token);  // Convert the second token to long
    }
  }

  snprintf(buffer, BUFFER_SIZE, "Recieved: %d,%ld", yaw_, distance_);
  DEBUG_PRINT(DEBUG_COMM, buffer);
}

void telemetryPacket::readUartBytes() {
  constexpr uint8_t bytesToRead = sizeof(yaw_) + sizeof(distance_);
  uint8_t buffer[bytesToRead];

  uint8_t bytesRead = 0;
  while (Serial.available() > 0 && bytesRead < bytesToRead) {
    buffer[bytesRead++] = Serial.read();
  }

  if (bytesRead < bytesToRead) {
    DEBUG_PRINT(DEBUG_COMM, "Insufficient UART data received.");
    return;
  }

  memcpy(&yaw_, buffer, sizeof(yaw_));
  memcpy(&distance_, buffer + sizeof(yaw_), sizeof(distance_));

  // DEBUG_PRINT(DEBUG_COMM, "Recv UART: " + String(yaw_) + "," + String(distance_));
}

void telemetryPacket::readUartASCII() {
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) {
    DEBUG_PRINT(DEBUG_COMM, "No UART input received.");
    return;
  }
  int yaw_Index = input.indexOf("yaw,");
  int distIndex = input.indexOf(",distance_,");
  if (yaw_Index != -1 && distIndex != -1) {
    yaw_ = input.substring(yaw_Index + 4, distIndex).toFloat();
    distance_ = input.substring(distIndex + 10).toFloat();
  }
  DEBUG_PRINT(DEBUG_COMM, "UART Yaw: " + String(yaw_));
  DEBUG_PRINT(DEBUG_COMM, "UART Distance: " + String(distance_));
}


struct commandPacket {
  TumblerCmd command_;
  int16_t value_ = 0.0;
  void sendI2CBytes(uint8_t address) const;
  void sendI2CASCII(uint8_t address) const;
  void sendUartBytes() const;
  void sendUartASCII() const;
  void readI2CBytes(uint8_t address);
  void readI2CASCII(uint8_t address);
  void readUartBytes();
  void readUartASCII();
};


void commandPacket::sendI2CBytes(uint8_t address) const {
  Wire.beginTransmission(address);
  Wire.write(command_);
  Wire.write(reinterpret_cast<const uint8_t*>(&value_), sizeof(value_));
  uint8_t result = Wire.endTransmission();

#if DEBUG_COMM
  if (result != 0) {
    DEBUG_PRINT(DEBUG_COMM, "Error: I2C transmission failed with code " + String(result));
  } else {
    String msg = "Sent I2C: " + commandToString(command_) + "," + String(value_);
    DEBUG_PRINT(DEBUG_COMM, msg);
  }
#endif
}


void commandPacket::sendI2CASCII(uint8_t address) const {
  Wire.beginTransmission(address);
  char packet[BUFFER_SIZE];
  snprintf(packet, BUFFER_SIZE, "%d,%d", command_, value_);
  Wire.print(packet);
  uint8_t result = Wire.endTransmission();
  if (result != 0) {
    DEBUG_PRINT(DEBUG_COMM, "Error: I2C transmission failed with code " + String(result));
  } else {
    DEBUG_PRINT(DEBUG_COMM, "I2C Command: " + String(packet));
  }
}


void commandPacket::sendUartBytes() const {
  constexpr uint8_t bytesToSend = sizeof(command_) + sizeof(value_);
  uint8_t buffer[bytesToSend];
  memcpy(buffer, &command_, sizeof(command_));
  memcpy(buffer + sizeof(command_), &value_, sizeof(value_));
  Serial.write(buffer, bytesToSend);

  // DEBUG_PRINT(DEBUG_COMM, "Sent UART: " + String(static_cast<uint8_t>(command_)) + "," + String(value_));
}


void commandPacket::sendUartASCII() const {
  Serial.println(command_ + "," + String(value_));
}


void commandPacket::readI2CBytes(uint8_t numBytes) {
  if (numBytes < 1) {
    Serial.println("Error: No data received.");
    return;
  }

  // Read the first byte as the command
  command_ = static_cast<TumblerCmd>(Wire.read());

  // Check for valid command
  if (command_ >= INVALID) {
    command_ = INVALID;  // Default to Stop
    DEBUG_PRINT(DEBUG_COMM, "Error: Invalid command received.");
    return;
  }

  Wire.readBytes(reinterpret_cast<char*>(&value_), sizeof(int));

  DEBUG_PRINT(DEBUG_COMM, "Recv I2C: " + commandToString(command_) + "," + String(value_));
}


void commandPacket::readI2CASCII(uint8_t numBytes) {
  if (Wire.available() < BUFFER_SIZE) {
    DEBUG_PRINT(DEBUG_COMM, "Insufficient I2C data received.");
    return;
  }

  char cmdByte = Wire.read();
  command_ = stringToCommand(String(cmdByte));

  uint8_t valueBytes[4];
  for (int i = 0; i < 4; i++) {
    valueBytes[i] = Wire.read();
  }

  float* valuePtr = reinterpret_cast<float*>(valueBytes);
  value_ = *valuePtr;

  DEBUG_PRINT(DEBUG_COMM, "I2C Command: " + command_);
  DEBUG_PRINT(DEBUG_COMM, "I2C Value: " + String(value_));
}


void commandPacket::readUartBytes() {
  // Read the first byte to get the command
  if (Serial.available() < 2) {
    DEBUG_PRINT(DEBUG_COMM, "Insufficient data received.");
    return;
  }

  command_ = static_cast<TumblerCmd>(Serial.read());

  uint8_t valueBytes[4];
  for (int i = 0; i < 4 && Serial.available(); i++) {
    valueBytes[i] = Serial.read();
  }

  float* valuePtr = reinterpret_cast<float*>(valueBytes);
  value_ = *valuePtr;

  DEBUG_PRINT(DEBUG_COMM, "Command: " + command_);
  DEBUG_PRINT(DEBUG_COMM, "Value: " + String(value_));
}


void commandPacket::readUartASCII() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (!input.length()) {
      DEBUG_PRINT(DEBUG_COMM, "No input received.");
      command_ = INVALID;
      return;
    }

    int commaIndex = input.indexOf(',');
    if (commaIndex == -1) {
      DEBUG_PRINT(DEBUG_COMM, "Comma not found in input.");
      command_ = INVALID;
      return;
    }

    String cmdStr = input.substring(0, commaIndex);
    cmdStr.trim();

    String valStr = input.substring(commaIndex + 1);
    valStr.trim();

    if (!cmdStr.length()) {
      DEBUG_PRINT(DEBUG_COMM, "Empty command.");
      command_ = INVALID;
      return;
    }

    command_ = stringToCommand(cmdStr);

    if (isNumeric(valStr)) {
      value_ = valStr.toInt();
    } else {
      DEBUG_PRINT(DEBUG_COMM, "Invalid value.");
      value_ = 0.0;
    }

    DEBUG_PRINT(DEBUG_COMM, "Recv UART: " + commandToString(command_) + "," + String(value_));
  }
}


bool isNumeric(const String& str) {
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
