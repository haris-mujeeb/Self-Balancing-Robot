#define BUFFER_SIZE 255

char inputBuffer[BUFFER_SIZE];
uint8_t bufferIndex = 0;

void setup() {
  // Initialize UART communication with 115200 baud rate
  Serial.begin(115200);
  while (!Serial) {
    // Wait for the Serial port to be ready (needed for some boards like Arduino Leonardo)
  } 

  // Serial.println("UART initialized with 115200-8N1 settings.");
  Serial.println("Serial initialized.");
}

void loop() {
  if (Serial.available()) {
    char incomingChar = Serial.read();
    if(incomingChar == 'n'){      
      // End of message; process the buffer
      inputBuffer[bufferIndex] = '\0'; // Null-terminate the string
      String data = String(inputBuffer);
      Serial.println(data);           // Echo the received data
      parseAndCalculate(data);        // Parse and calculate
      bufferIndex = 0;                // Reset buffer index for the next messa
    } else {
      // Store the character in the buffer if space allows
      if (bufferIndex < BUFFER_SIZE - 1) {
        inputBuffer[bufferIndex++] = incomingChar;
      } else {
        // Handle buffer overflow (optional)
        Serial.println("Buffer overflow! Message too long.");
        bufferIndex = 0; // Reset buffer index
      }
    }
    Serial.println(inputBuffer); // Send to Serial
    // parseAndCalculate(data);
  }
}

void parseAndCalculate(String input) {
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
  float forwardAvg = sensorData[0];
  float leftAvg    = sensorData[2];
  float backAvg    = sensorData[4];
  float rightAvg   = sensorData[6];

  // Print results
  Serial.print("Forward Avg: "); Serial.println(forwardAvg);
  Serial.print("Right Avg: "); Serial.println(rightAvg);
  Serial.print("Back Avg: "); Serial.println(backAvg);
  Serial.print("Left Avg: "); Serial.println(leftAvg);
}