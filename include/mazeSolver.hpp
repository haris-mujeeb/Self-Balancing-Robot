#include "HardwareSerial.h"
#include "debugConfig.h"

#if DEBUG_TERRA_RANGER
  String debugMsg = "";
#endif

#define BUFFER_SIZE 100 // Define the maximum buffer size

class mazeSolver
{
private:
  void parseAndCalculate(String input);
  char inputBuffer[BUFFER_SIZE]; // Buffer to store incoming data
  int bufferIndex = 0;           // Current index in the buffer
  float fwdDist, backDist, leftDist, rightDist;

public:
  mazeSolver();
  void getTerraData();
};

mazeSolver::mazeSolver(/* args */) {
  // Serial.begin(115200);  
  #if DEBUG_TERRA_RANGER
    debugMsg = "Serial initialized for TerraRanger.";
  #endif
}


void mazeSolver::getTerraData(){
  if(Serial.available()) {

    char incomingChar = Serial.read();
    if(incomingChar == '\n'){      
      // End of message; process the buffer
      inputBuffer[bufferIndex] = '\0'; // Null-terminate the string
      String data = String(inputBuffer);
      DEBUG_PRINT(DEBUG_TERRA_RANGER, data);        // Echo the received data
      parseAndCalculate(data);        // Parse and calculate
      bufferIndex = 0;                // Reset buffer index for the next message
    } else {
      // Store the character in the buffer if space allows
      if (bufferIndex < BUFFER_SIZE - 1) {
        inputBuffer[bufferIndex++] = incomingChar;
      } else {
        inputBuffer[0] = '\0';          // Clear the buffer
        DEBUG_PRINT(DEBUG_TERRA_RANGER, "overflow! Message too long.\n");
        bufferIndex = 0; // Reset buffer index
      }
    }
  }
}

void mazeSolver::parseAndCalculate(String input) {
  // Check if the input starts with "MF\t"
  if (!input.startsWith("MF\t")) {
      return; // Skip processing if the prefix is not found
  }

  // Split the data string into tokens
  int sensorData[8] = {0}; // Array to hold sensor values
  int index = 0;
  int startIndex = input.indexOf('\t') + 1; // Start after "MF\t"

  // Parse the input string in a single loop
  while (startIndex > 0 && index < 8) {
    int nextTab = input.indexOf('\t', startIndex);
    String value = (nextTab > 0) ? input.substring(startIndex, nextTab) : input.substring(startIndex);
    sensorData[index++] = value.toInt(); // Convert to integer
    startIndex = nextTab + 1; // Move to next value
  }

  // Calculate averages for each direction
  fwdDist = sensorData[0];
  leftDist = sensorData[2];
  backDist = sensorData[4];
  rightDist = sensorData[6];

  #if DEBUG_TERRA_RANGER
    debugMsg += "Fwd: "; debugMsg += String(fwdDist);
    debugMsg +="Left: "; debugMsg += String(leftDist);
    debugMsg +="Back: "; debugMsg += String(backDist);
    debugMsg +="Right: "; debugMsg += String(rightDist);
    DEBUG_PRINT(DEBUG_TERRA_RANGER, debugMsg);
    debugMsg = "";
  #endif
}
