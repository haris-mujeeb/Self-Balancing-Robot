/**
 * @file TB6612FNG.hpp
 * @brief Driver class for the TB6612FNG motor driver, supporting multiple motor control configurations.
 *
 * This header file provides an abstraction for controlling motors using the TB6612FNG driver with flexible
 * compile-time configuration for different control modes. It supports single-direction, inverted pins, and
 * four-pin configurations for motor control.
 * 
 * @note Ensure that one of the configuration modes (`MOTOR_MODE_SINGLE_DIRECTION`, `MOTOR_MODE_INVERTED_PINS`, 
 * or `MOTOR_MODE_FOUR_PINS`) is defined at compile time.
 */

#ifndef TB6612FNG_H
#define TB6612FNG_H

#include <Arduino.h>
#include "pins.hpp"
#include "debugConfig.h"

// Compile-time configuration modes
// Uncomment one of the following to enable the desired motor control mode:
// #define MOTOR_MODE_SINGLE_DIRECTION  ///< Use two pins per motor (single direction mode).
#define MOTOR_MODE_INVERTED_PINS       ///< Use inverted pins for motor control.
                                       ///< Typically requires three pins per motor.
// #define MOTOR_MODE_FOUR_PINS         ///< Use four pins per motor for advanced control.

class TB6612FNG {
private:
    // Motor driver pin definitions based on selected mode
#if defined(MOTOR_MODE_SINGLE_DIRECTION)
    uint16_t stbyPin;     ///< Standby pin.
    uint16_t ain1Pin;     ///< Motor A direction control pin 1.
    uint16_t ain2Pin;     ///< Motor A direction control pin 2.
    uint16_t pwmaPin;     ///< Motor A PWM control pin.
#elif defined(MOTOR_MODE_INVERTED_PINS)
    uint16_t stbyPin;     ///< Standby pin.
    uint16_t ain1Pin;     ///< Motor A direction control pin.
    uint16_t bin1Pin;     ///< Motor B direction control pin.
    uint16_t pwmaPin;     ///< Motor A PWM control pin.
    uint16_t pwmbPin;     ///< Motor B PWM control pin.
#elif defined(MOTOR_MODE_FOUR_PINS)
    uint16_t stbyPin;     ///< Standby pin.
    uint16_t ain1Pin;     ///< Motor A direction control pin 1.
    uint16_t ain2Pin;     ///< Motor A direction control pin 2.
    uint16_t bin1Pin;     ///< Motor B direction control pin 1.
    uint16_t bin2Pin;     ///< Motor B direction control pin 2.
    uint16_t pwmaPin;     ///< Motor A PWM control pin.
    uint16_t pwmbPin;     ///< Motor B PWM control pin.
#else
    #error "No motor mode defined. Please define one of MOTOR_MODE_SINGLE_DIRECTION, MOTOR_MODE_INVERTED_PINS, or MOTOR_MODE_FOUR_PINS."
#endif

public:
    // Constructors for the motor driver, tailored to the selected configuration
#if defined(MOTOR_MODE_SINGLE_DIRECTION)
    /**
     * @brief Constructor for single-direction mode.
     * 
     * @param stby Standby pin.
     * @param ain1 Motor A direction control pin 1.
     * @param ain2 Motor A direction control pin 2.
     * @param pwma Motor A PWM control pin.
     */
    TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t ain2, uint16_t pwma) {
        stbyPin = stby; ain1Pin = ain1; ain2Pin = ain2; pwmaPin = pwma;
    }
#endif

#if defined(MOTOR_MODE_INVERTED_PINS)
    /**
     * @brief Constructor for inverted pin mode.
     * 
     * @param stby Standby pin.
     * @param ain1 Motor A direction control pin.
     * @param bin1 Motor B direction control pin.
     * @param pwma Motor A PWM control pin.
     * @param pwmb Motor B PWM control pin.
     */
    TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t bin1, uint16_t pwma, uint16_t pwmb) {
        stbyPin = stby; ain1Pin = ain1; bin1Pin = bin1;
        pwmaPin = pwma; pwmbPin = pwmb;
    }
#endif

#if defined(MOTOR_MODE_FOUR_PINS)
    /**
     * @brief Constructor for four-pin mode.
     * 
     * @param stby Standby pin.
     * @param ain1 Motor A direction control pin 1.
     * @param ain2 Motor A direction control pin 2.
     * @param bin1 Motor B direction control pin 1.
     * @param bin2 Motor B direction control pin 2.
     * @param pwma Motor A PWM control pin.
     * @param pwmb Motor B PWM control pin.
     */
    TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t ain2, 
              uint16_t bin1, uint16_t bin2, uint16_t pwma, uint16_t pwmb) {
        stbyPin = stby;
        ain1Pin = ain1; ain2Pin = ain2;
        bin1Pin = bin1; bin2Pin = bin2;
        pwmaPin = pwma; pwmbPin = pwmb;
    }
#endif

    /**
     * @brief Initializes motor driver pins for operation.
     */
    void init() {
        pinMode(stbyPin, OUTPUT);
        pinMode(ain1Pin, OUTPUT);
        pinMode(pwmaPin, OUTPUT);

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
        DEBUG_PRINT(DEBUG_MOTOR, "Motors initialized.");
    }

    /**
     * @brief Controls motor A with a specified speed and direction.
     * 
     * @param speed Speed of the motor (-255 to 255).
     */
    inline void motorA(int16_t speed) {
        bool clockwise = speed > 0;
        speed = constrain(speed, -255, 255);
        speed = abs(speed);
        digitalWrite(ain1Pin, clockwise ? LOW : HIGH);
        #if TWO_PINS_SINGLE_MOTOR
        digitalWrite(ain2Pin, clockwise ? HIGH : LOW);
        #endif
        analogWrite(pwmaPin, speed);
        digitalWrite(stbyPin, HIGH);
        DEBUG_PRINT(DEBUG_MOTOR, "[speed: " + String(speed) + "] MotorA moving " + (clockwise ? "Clockwise" : "Counter-clockwise"));
    }

    /**
     * @brief Controls motor B with a specified speed and direction.
     * 
     * @param speed Speed of the motor (-255 to 255).
     */
    inline void motorB(int16_t speed) {
        bool clockwise = speed > 0;
        speed = constrain(speed, -255, 255);
        speed = abs(speed);
        #if defined(MOTOR_MODE_INVERTED_PINS)
        digitalWrite(bin1Pin, clockwise ? LOW : HIGH);
        #elif defined(MOTOR_MODE_FOUR_PINS)
        digitalWrite(bin1Pin, clockwise ? LOW : HIGH);
        digitalWrite(bin2Pin, clockwise ? HIGH : LOW);
        #endif
        analogWrite(pwmbPin, speed);
        digitalWrite(stbyPin, HIGH);
        DEBUG_PRINT(DEBUG_MOTOR, "[speed: " + String(speed) + "] MotorB moving " + (clockwise ? "Clockwise" : "Counter-clockwise"));
    }

    /**
     * @brief Stops both motors by disabling the standby pin and PWM outputs.
     */
    inline void stop() {
        digitalWrite(stbyPin, LOW);
        analogWrite(pwmaPin, 0);
        #ifdef MOTOR_MODE_INVERTED_PINS
        analogWrite(pwmbPin, 0);
        #elif defined(MOTOR_MODE_FOUR_PINS)
        analogWrite(pwmbPin, 0);
        #endif
        DEBUG_PRINT(DEBUG_MOTOR, "Motors stopped.");
    }

    /**
     * @brief Moves both motors at the specified speed.
     * 
     * @param speed Speed of both motors (-255 to 255).
     */
    inline void move(int16_t speed) {
        if (speed == 0) {
            stop();
        } else {
            motorA(speed);
            #if defined(MOTOR_MODE_INVERTED_PINS) || defined(MOTOR_MODE_FOUR_PINS)
            motorB(speed);
            #endif
        }
    }
};

extern TB6612FNG motors;

#endif // TB6612FNG_H