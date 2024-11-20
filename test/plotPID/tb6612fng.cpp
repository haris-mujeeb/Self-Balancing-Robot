#include "tb6612fng.hpp"

void TB6612FNG::init() {
  pinMode(stbyPin, OUTPUT);
  pinMode(ain1Pin, OUTPUT);
  pinMode(pwmaPin, OUTPUT);

  // Compile-time configuration branching
  #if defined(MOTOR_MODE_SINGLE_DIRECTION)
      pinMode(ain2Pin, OUTPUT);
  #elif defined(MOTOR_MODE_INVERTED_PINS)
      pinMode(bin1Pin, OUTPUT);
      pinMode(pwmbPin, OUTPUT);
  #elif defined(MOTOR_MODE_FOUR_PINS)
      pinMode(ain2Pin, OUTPUT);
      pinMode(bin1Pin, OUTPUT);
      pinMode(bin2Pin, OUTPUT);
      pinMode(pwmbPin, OUTPUT);
  #endif
  DEBUG_PRINT(DEBUG_MOTOR, "[Debug] Motors initialized.");
}


// Motor control methods with compile-time optimization
void TB6612FNG::motorA(uint8_t speed, bool clockwise) {
  digitalWrite(ain1Pin, clockwise ? LOW : HIGH);
#if TWO_PINS_SINGLE_MOTOR
  digitalWrite(ain2Pin, clockwise ? HIGH : LOW);
#endif
  analogWrite(pwmaPin, speed);
  digitalWrite(stbyPin, HIGH);
  DEBUG_PRINT(DEBUG_MOTOR, "[Debug] [speed: " + String(speed) 
    + "] MotorA moving " + (clockwise ? "Clockwise" : "Counter-clockwise"));
}


void TB6612FNG::motorB(uint8_t speed, bool clockwise) {
  #if defined(MOTOR_MODE_INVERTED_PINS)
      digitalWrite(bin1Pin, clockwise ? LOW : HIGH);
  #elif defined(MOTOR_MODE_FOUR_PINS)
      digitalWrite(bin1Pin, clockwise ? LOW : HIGH);
      digitalWrite(bin2Pin, clockwise ? HIGH : LOW);
  #endif
  analogWrite(pwmbPin, speed);
  digitalWrite(stbyPin, HIGH);
    DEBUG_PRINT(DEBUG_MOTOR, "[Debug] [speed: " + String(speed)
      + "] MotorB moving " + (clockwise ? "Clockwise" : "Counter-clockwise"));
}

inline void TB6612FNG::stop() {
  digitalWrite(stbyPin, LOW);
  analogWrite(pwmaPin, 0);
#ifdef MOTOR_MODE_INVERTED_PINS
  analogWrite(pwmbPin, 0);
#elif defined(MOTOR_MODE_FOUR_PINS)
  analogWrite(pwmbPin, 0);
#endif
  DEBUG_PRINT(DEBUG_MOTOR, "[Debug] Motors stopped.");
}


void TB6612FNG::move(unsigned char speed, bool clockwise) {
  if (speed == 0) {
    stop();
  } else {
    motorA(speed, clockwise);
#ifdef MOTOR_MODE_INVERTED_PINS || MOTOR_MODE_FOUR_PINS
    motorB(speed, clockwise);
#endif
  }
}

void TB6612FNG::moveCCW(unsigned char speed) { move(speed, false); }
void TB6612FNG::moveCW(unsigned char speed) { move(speed, true); }
void TB6612FNG::moveForw(unsigned char speed) { move(speed, false); }
void TB6612FNG::moveBack(unsigned char speed) { move(speed, true); }