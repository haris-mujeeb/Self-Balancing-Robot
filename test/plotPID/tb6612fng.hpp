#ifndef TB6612FNG_H
#define TB6612FNG_H
#include <Arduino.h>
#include "pins.hpp"
#include "debugConfig.h"

// Compile-time configuration modes
// #define MOTOR_MODE_SINGLE_DIRECTION
#define MOTOR_MODE_INVERTED_PINS
// #define MOTOR_MODE_FOUR_PINS

class TB6612FNG {
private:
// Compile-time configuration flags
#if defined(MOTOR_MODE_SINGLE_DIRECTION)
  uint16_t stbyPin;
  uint16_t ain1Pin, ain2Pin;
  uint16_t pwmaPin;
#elif defined(MOTOR_MODE_INVERTED_PINS)
  uint16_t stbyPin;
  uint16_t ain1Pin, bin1Pin;
  uint16_t pwmaPin, pwmbPin;
#elif defined(MOTOR_MODE_FOUR_PINS)
  uint16_t stbyPin;
  uint16_t ain1Pin, ain2Pin;
  uint16_t bin1Pin, bin2Pin;
  uint16_t pwmaPin, pwmbPin;
#else
  #error "No motor mode defined. Please define one of MOTOR_MODE_SINGLE_DIRECTION, MOTOR_MODE_INVERTED_PINS, or MOTOR_MODE_FOUR_PINS."
#endif

public:
  // Constructors with compile-time configuration
#if defined(MOTOR_MODE_SINGLE_DIRECTION)
  TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t ain2, uint16_t pwma) {
      stbyPin = stby; ain1Pin = ain1; ain2Pin = ain2; pwmaPin = pwma;
  }
#endif

#if defined(MOTOR_MODE_INVERTED_PINS)
  TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t bin1, uint16_t pwma, uint16_t pwmb) {
      stbyPin = stby; ain1Pin = ain1; bin1Pin = bin1;
      pwmaPin = pwma; pwmbPin = pwmb;
  }
#endif

#if defined(MOTOR_MODE_FOUR_PINS)
  TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t ain2, 
              uint16_t bin1, uint16_t bin2, uint16_t pwma, uint16_t pwmb) {
      stbyPin = stby;
      ain1Pin = ain1; ain2Pin = ain2;
      bin1Pin = bin1; bin2Pin = bin2;
      pwmaPin = pwma; pwmbPin = pwmb;
  }
#endif

  // Core method declarations with compile-time optimizations
  void init();
  void motorA(int16_t speed);
  void motorB(int16_t speed);
  void move(int16_t speed);
  void stop();
};

#endif // TB6612FNG_H