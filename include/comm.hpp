#include "Arduino.h"

bool isNumeric(String str);

struct telemetryPacket {
  float yaw;
  float distance;
  void sendPacketBytes();
  void sendPacketASCII();
};

// Function to send packet as raw bytes (binary)
void telemetryPacket::sendPacketBytes(){
    Serial.write(reinterpret_cast<uint8_t*>(&yaw), sizeof(float));   // Send yaw as 4 bytes
    Serial.write(reinterpret_cast<uint8_t*>(&distance), sizeof(float)); // Send distance as 4 bytes
}

// Function to send packet as ASCII (human-readable)
void telemetryPacket::sendPacketASCII(){
      // Send yaw in ASCII (converted to string)
    Serial.print("yaw:");
    Serial.println(yaw, 6); // Print float with 6 decimal places

    // Send distance in ASCII (converted to string)
    Serial.print("distance:");
    Serial.println(distance, 6); // Print float with 6 decimal places
}



struct telemetryCommands {
  char command;
  int value;
  void readPacketBytes();
  void readPacketASCII();
};

void telemetryCommands::readPacketBytes(){
  // Read the first byte to get the command
  if (Serial.available() > 0) {
    command = (char)Serial.read();  // Read the command byte
  }
  value = 0;                    // Initialize value to 0
  
  // Read the remaining bytes which represent the value (e.g., 2000)
  if (Serial.available() > 0) {
    while (Serial.available() > 0) {
      char digit = (char)Serial.read();  // Read the next byte
      if (digit >= '0' && digit <= '9') {
        value = value * 10 + (digit - '0');  // Convert ASCII digit to integer
      }
      else {
        break;  // Stop if we encounter a non-digit (e.g., end of number)
      }
    }
  }
}

void telemetryCommands::readPacketASCII(){
  // Read a full command from the serial input
  String input = Serial.readStringUntil('\n'); // Read until newline
 
  // Remove any trailing '\r' (carriage return)
  input.trim(); // Removes leading/trailing whitespace, including '\r'

  if (input.length() > 0) {
    // Extract the command (first character) and the value (remaining characters)
    command = input.charAt(0);
    String valueString = input.substring(1);  
    if (valueString.length() > 0 && isNumeric(valueString)){
      value = valueString.toInt();   // Convert the rest to an integer
    } else {
      // Handle invalid value error
      DEBUG_PRINT(DEBUG_COMM,"Invalid value.");
      value = 0;
    }
  } else {
    DEBUG_PRINT(DEBUG_COMM,"No input received.");
    command = '\0';
  }
}

bool isNumeric(String str) {
  for(unsigned int i = 0; i < str.length(); i++) {
    if(!isDigit(str[i])  &&  !(i == 0 && str[i] == '-')) {
      return false;
    }
  }
  return true;
}
