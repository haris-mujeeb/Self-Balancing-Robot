 #include "tb6612fng.h"

#define AIN1 7
#define PWMA_LEFT 5
#define BIN1 12
#define PWMB_RIGHT 6
#define STBY_PIN 8


TB6612FNG motorController(STBY_PIN, AIN1, BIN1, PWMA_LEFT, PWMB_RIGHT);

void setup(){
  Serial.begin(9600);
  motorController.init();
  motorController.move(100, 1);
 }

void loop(){
}