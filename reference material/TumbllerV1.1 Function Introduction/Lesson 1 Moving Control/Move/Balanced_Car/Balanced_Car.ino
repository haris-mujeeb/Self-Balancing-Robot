#include "Motor.h"


Motor Motor;

/*You can set the speed: 0~255 */
#define SPEED 150

char strbuf[][50]={
                    "Forward:encoder_count_right_a:",
                    "Forward:encoder_count_left_a:",
                    "Back:encoder_count_right_a:",
                    "Back:encoder_count_left_a:",
                    "Left:encoder_count_right_a:",
                    "Left:encoder_count_left_a:",
                    "Right:encoder_count_right_a:",
                    "Right:encoder_count_left_a:"
                   };

void setup() 
{
  Motor.Pin_init();
  Motor.Encoder_init();
  Serial.begin(9600);
}

void loop() {
    
   for(int i = 0,j=0; i < 4,j<8; i++)
  {
    (Motor.*(Motor.MOVE[i]))(SPEED);
    delay(2000);
    Serial.print(strbuf[j++]);
    Serial.println(Motor.encoder_count_right_a);
    Serial.print(strbuf[j++]);
    Serial.println(Motor.encoder_count_left_a);
    Motor.encoder_count_right_a=0;
    Motor.encoder_count_left_a=0;
    }
}
