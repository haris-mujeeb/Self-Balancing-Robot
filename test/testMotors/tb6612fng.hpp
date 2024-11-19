/**
 * @file TB6612FNG.h
 * @brief Driver for the TB6612FNG motor driver.
 * 
 * This library allows control of motors using the TB6612FNG motor driver.
 * It supports multiple configurations for driving DC motors with various modes,
 * including single-directional, inverted pins bi-directional, and four pins bi-directional.
 * 
 * @note The TB6612FNG must be properly wired to the microcontroller for this library to function.
 */

#pragma once

#include "Arduino.h"

/**
 * @enum TB6612FNGMode
 * @brief Enumeration for motor driver modes.
 * 
 * This enum defines the various operational modes of the TB6612FNG motor driver.
 */
enum TB6612FNGMode {
    TWO_PINS_SINGLE_MOTOR = 1,        ///< Single direction control mode
    INVERTED_PINS_TWO_MOTORS = 2, ///< Inverted pins bi-directional mode
    FOUR_PINS_TWO_MOTORS = 4    ///< Four pins bi-directional mode
};

/**
 * @class TB6612FNG
 * @brief Class for controlling TB6612FNG motor driver.
 * 
 * This class provides an interface to control the TB6612FNG motor driver,
 * allowing users to control motor direction, speed, and braking.
 */
class TB6612FNG {
private:
    TB6612FNGMode mode;           ///< Current operating mode of the motor driver
    uint16_t stbyPin;             ///< Standby pin number
    uint16_t pwmaPin;             ///< PWM pin for motor A
    uint16_t pwmbPin;             ///< PWM pin for motor B
    uint16_t ain1Pin;             ///< Input 1 pin for motor A
    uint16_t ain2Pin;             ///< Input 2 pin for motor A
    uint16_t bin1Pin;             ///< Input 1 pin for motor B
    uint16_t bin2Pin;             ///< Input 2 pin for motor B

public:
    /**
     * @brief Default constructor.
     * 
     * Initializes the motor driver in an undefined state.
     */
    TB6612FNG();

    /**
     * @brief Constructor for single-direction control.
     * 
     * @param stby Standby pin number
     * @param ain1 Input 1 pin for motor A
     * @param ain2 Input 2 pin for motor A
     * @param pwma PWM pin for motor A
     */
    TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t ain2, uint16_t pwma);

    /**
     * @brief Constructor for full H-bridge control (2-pin bidirectional).
     * 
     * @param stby Standby pin number
     * @param ain1 Input 1 pin for motor A
     * @param bin1 Input 1 pin for motor B
     * @param pwma PWM pin for motor A
     * @param pwmb PWM pin for motor B
     */
    TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t bin1, uint16_t pwma, uint16_t pwmb);

    /**
     * @brief Constructor for full H-bridge control (4-pin bidirectional).
     * 
     * @param stby Standby pin number
     * @param ain1 Input 1 pin for motor A
     * @param ain2 Input 2 pin for motor A
     * @param bin1 Input 1 pin for motor B
     * @param bin2 Input 2 pin for motor B
     * @param pwma PWM pin for motor A
     * @param pwmb PWM pin for motor B
     */
    TB6612FNG(uint16_t stby, uint16_t ain1, uint16_t ain2, uint16_t bin1, uint16_t bin2, uint16_t pwma, uint16_t pwmb);

    /**
     * @brief Initializes the motor driver pins.
     * 
     * This method sets the pin modes for the TB6612FNG motor driver based on the selected mode.
     */
    void init();

    /**
     * @brief Gets the current mode of the motor driver.
     * 
     * @return The current mode of the motor driver.
     */
    TB6612FNGMode getMode();

    // Motor control functions
    /**
     * @brief Puts the motor driver into standby mode.
     * 
     * This will disable the motor outputs and put the driver into a low-power state.
     */
    void standBy();

    /**
     * @brief Moves motorA in the clockwise direction.
     * 
     * @param speed The speed at which to move the motors (0-255).
     */
    void motorA(unsigned char speed, bool clokcwise);


    /**
     * @brief Moves motorB in the clockwise direction.
     * 
     * @param speed The speed at which to move the motors (0-255).
     */
    void motorB(unsigned char speed, bool clokcwise);

    /**
     * @brief Moves motorB in the clockwise direction.
     * 
     * @param speed The speed at which to move the motors (0-255).
     */
    void move(unsigned char speed, bool clokcwise);


    /**
     * @brief Applies a short brake to the motors.
     * 
     * This function should be implemented to provide a braking mechanism.
     */
    void shortBrake();

    /**
     * @brief Moves both motors in the counter-clockwise direction.
     * 
     * @param speed The speed at which to move the motors (0-255).
     */
    void moveCCW(unsigned char speed);

    /**
     * @brief Moves both motors in the clockwise direction.
     * 
     * @param speed The speed at which to move the motors (0-255).
     */
    void moveCW(unsigned char speed);

    /**
     * @brief Moves both motors forward (equivalent to clockwise).
     * 
     * @param speed The speed at which to move the motors (0-255).
     */
    void moveForw(unsigned char speed);

    /**
     * @brief Moves both motors backward (equivalent to counter-clockwise).
     * 
     * @param speed The speed at which to move the motors (0-255).
     */
    void moveBack(unsigned char speed);

    /**
     * @brief Stops the motors (open circuit).
     * 
     * This method will stop the motor rotation and disable the outputs.
     */
    void stop();
};