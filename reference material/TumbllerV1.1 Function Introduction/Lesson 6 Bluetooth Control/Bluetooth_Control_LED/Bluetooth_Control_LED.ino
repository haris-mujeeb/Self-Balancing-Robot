#include "rgb.h"
#include "MsTimer2.h"

void setup() {
  Serial.begin(9600);
  rgb.initialize();
}

void loop() {
  rgb.Bluetooth_Data_Processing();
  }


  
