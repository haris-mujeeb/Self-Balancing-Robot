#include "tb6612fng.hpp"

// Constructor for 1 motor controlled with 2 pins 
TB6612FNG::TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t ain2, uint16_t pwma) {
  stbyPin = stby;
  ain1Pin = ain1;
  ain2Pin = ain2;
  pwmaPin = pwma;
  mode = TWO_PINS_SINGLE_MOTOR;  
}


// Constructor for 2 motors controlled with 2 inverted pins 
TB6612FNG::TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t bin1, uint16_t pwma, uint16_t pwmb) {
  stbyPin = stby;
  ain1Pin = ain1;
  bin1Pin = bin1;
  pwmaPin = pwma;
  pwmbPin = pwmb;
  mode = INVERTED_PINS_TWO_MOTORS;  
}

// Constructor for 2 motors controlled with 4 pins 
TB6612FNG::TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t ain2, uint16_t bin1, uint16_t bin2, uint16_t pwma, uint16_t pwmb) {
  stbyPin = stby;
  ain1Pin = ain1;
  ain2Pin = ain2;
  bin1Pin = bin1;
  bin2Pin = bin2;
  pwmaPin = pwma;
  pwmbPin = pwmb;
  mode = FOUR_PINS_TWO_MOTORS;  
}


void TB6612FNG::init(){
  pinMode(stbyPin, OUTPUT);
  pinMode(ain1Pin, OUTPUT);
  pinMode(pwmaPin, OUTPUT);
  switch (mode) {
   case INVERTED_PINS_TWO_MOTORS:
    pinMode(bin1Pin, OUTPUT);
    pinMode(pwmbPin, OUTPUT);
    break;
   case FOUR_PINS_TWO_MOTORS:
    pinMode(ain2Pin, OUTPUT);
    pinMode(bin1Pin, OUTPUT);
    pinMode(bin2Pin, OUTPUT);
    pinMode(pwmbPin, OUTPUT);
    break;
   default:
    pinMode(ain2Pin, OUTPUT); // Set for single-directional control
    break;
  }
  DEBUG_PRINT(DEBUG_MOTOR, "[Debug] Motors initialized.");
}


TB6612FNGMode TB6612FNG::getMode() { return mode; }


inline void TB6612FNG::standBy() {  digitalWrite(stbyPin, LOW); } // Enable standby


void TB6612FNG::motorA(unsigned char speed, bool clockwise) { 
  // Control the direction and speed of motor A
  digitalWrite(ain1Pin, clockwise ? LOW : HIGH);
  if (mode != TWO_PINS_SINGLE_MOTOR) {
    digitalWrite(ain2Pin, clockwise ? HIGH : LOW);
  }
  analogWrite(pwmaPin, speed);
  digitalWrite(stbyPin, HIGH);
  String debugMsg = "[Debug] [speed: "+ String(speed) + "] MotorA moving" 
      + (clockwise ? "Clockwise" :  "Counter-clockwise");     
  DEBUG_PRINT(DEBUG_MOTOR, debugMsg);
}


void TB6612FNG::motorB(unsigned char speed, bool clockwise) {
    // Control the direction and speed of motor B
    if (mode == INVERTED_PINS_TWO_MOTORS) {
        digitalWrite(bin1Pin, clockwise ? LOW : HIGH);
        analogWrite(pwmbPin, speed);
    } else if (mode == FOUR_PINS_TWO_MOTORS) {
        digitalWrite(bin1Pin, clockwise ? LOW : HIGH);
        digitalWrite(bin2Pin, clockwise ? HIGH : LOW);
    }
    analogWrite(pwmbPin, speed);
    digitalWrite(stbyPin, HIGH);
    String debugMsg = "[Debug] [speed: "+ String(speed) + "] MotorB moving" 
      + (clockwise ? "Clockwise" :  "Counter-clockwise");     
    DEBUG_PRINT(DEBUG_MOTOR, debugMsg);
}

void TB6612FNG::move(unsigned char speed, bool clockwise) {
  if (speed == 0) {
      stop(); // Stop if speed is zero
  } else {
    motorA(speed, clockwise);
    motorB(speed, clockwise);
  }
}


// Stops the motors (open circuit)
inline void TB6612FNG::stop() {
  digitalWrite(stbyPin, LOW);  // Set standby
  analogWrite(pwmaPin, 0);      // Stop motor A
  analogWrite(pwmbPin, 0);      // Stop motor B
  DEBUG_PRINT(DEBUG_MOTOR, "[Debug] Motors stopped.");
}

// Moves both motors in the counter-clockwise direction
void TB6612FNG::moveCCW(unsigned char speed) {
  move(speed, false);  // Use move with clockwise = false
}

// Moves both motors in the clockwise direction
void TB6612FNG::moveCW(unsigned char speed) {
  move(speed, true);  // Use move with clockwise = false
}

void TB6612FNG::moveForw(unsigned char speed) {
  move(speed, false);  // Use move with clockwise = false
}

void TB6612FNG::moveBack(unsigned char speed) {
  move(speed, true);    // Backward is the same as clockwise in most cases
}
