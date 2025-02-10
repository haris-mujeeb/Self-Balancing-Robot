#include "Rgb.h"


void setup() { 
  rgb.initialize();
}


void loop() { 
  rgb.blink(150); // must be in the loop
  states();

  
}


unsigned long lastUpdateTimeMS = 0;
int count = 0;

void states(){
  if (millis() - lastUpdateTimeMS < 1000) {
    return;
  }

  lastUpdateTimeMS = millis();

  switch (count) {
    case 0:  
      rgb.flashGreenColorFront();
      break;
    case 1:
      rgb.flashYellowColorLeft();
      break;
    case 2:
      rgb.flashGreenColorBack();
      break;
    case 3:
      rgb.flashYellowColorRight();
      break;
  }
  count = (count + 1) % 4;;
}

