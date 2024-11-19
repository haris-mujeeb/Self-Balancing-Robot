#include "rgb.h"
#include "EnableInterrupt.h"

#define BUTTON 10
void setup() {
  rgb.initialize();
  enableInterrupt(BUTTON, Mode_KeyControl, FALLING );
}

void loop() {
 
}


void Mode_KeyControl()
{
  static int time_button,last_time_button;
  static int mode;
  time_button=millis();
  if(200 < (abs(time_button - last_time_button)))
  {
    mode++;
    switch(mode)
    {
      case 1:rgb.theaterChaseRainbow(50);break;
      case 2:rgb.rainbowCycle(20);break;
      case 3:rgb.theaterChase(127, 127, 127, 50);break;
      case 4:rgb.rainbow(20);break;
      case 5:rgb.whiteOverRainbow(20, 30, 4);break;
      case 6:rgb.rainbowFade2White(3, 50, 50); break;
      default:break;
    }
    if(mode>6)
      mode=0;
  }
}
