#include "rgb.h"



void setup() {
  rgb.initialize();

}

void loop() {
  
   rgb.theaterChaseRainbow(50);

   rgb.rainbowCycle(20);
  
   rgb.theaterChase(127, 127, 127, 50);
  
   rgb.rainbow(20);
  
   rgb.whiteOverRainbow(20, 30, 4);

   rgb.rainbowFade2White(3, 50, 50); 
}
