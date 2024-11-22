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
void TB6612FNG::motorA(int16_t speed) {
  bool clockwise = speed > 0 ? true : false;
  speed = constrain(speed, -255, 255);
  speed = static_cast<uint8_t>(abs(speed));
  digitalWrite(ain1Pin, clockwise ? LOW : HIGH);
#if TWO_PINS_SINGLE_MOTOR
  digitalWrite(ain2Pin, clockwise ? HIGH : LOW);
#endif
  analogWrite(pwmaPin, speed);
  digitalWrite(stbyPin, HIGH);
  DEBUG_PRINT(DEBUG_MOTOR, "[Debug] [speed: " + String(speed) 
    + "] MotorA moving " + (clockwise ? "Clockwise" : "Counter-clockwise"));
}


void TB6612FNG::motorB(int16_t speed) {
  bool clockwise = speed > 0 ? true : false;
  speed = constrain(speed, -255, 255);
  speed = static_cast<uint8_t>(abs(speed));
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


void TB6612FNG::move(int16_t speed) {
  if (speed == 0) {
    stop();
  } else {
    motorA(speed);
#if  defined(MOTOR_MODE_INVERTED_PINS) || defined(MOTOR_MODE_FOUR_PINS)
    motorB(speed);
#endif
  }
}
