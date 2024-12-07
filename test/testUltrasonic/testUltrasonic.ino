#include "ultrasonic.hpp"


unsigned long lastTime = 0;


void setup() {
  Serial.begin(9600);
  InitializeIRAndUltrasonic();
}


void loop() {
  CheckIRObstacle();
  StartUltrasonicMeasurement();

  // Test and Debugging
  if(millis() - lastTime > 500){
    lastTime = millis();
    irTest();
    usonicTest();
    printIRFilterValues();
  }
  delay(1);
}