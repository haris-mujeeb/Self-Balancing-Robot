/**
 * @file pins.hpp
 * @brief Pin definitions for hardware components of the self-balancing robot.
 * 
 * This file defines the pin connections used in the robot's hardware setup, 
 * including motor drivers, sensors, and other peripherals.
 */

#define VOL_MEASURE_PIN A2            ///< Analog pin for measuring battery voltage.
#define ECHO_PIN A3                   ///< Ultrasonic sensor echo pin.
#define TRIG_PIN 11                   ///< Ultrasonic sensor trigger pin.
#define RECV_PIN 9                    ///< IR receiver pin for remote control signals.
#define NUMPIXELS 4                   ///< Number of RGB LED pixels in the setup.
#define RGB_PIN 3                     ///< Digital pin for controlling the RGB LEDs.
#define AIN1 7                        ///< Motor driver input pin for motor A direction control.
#define PWMA_LEFT 5                   ///< PWM pin for motor A (left motor) speed control.
#define BIN1 12                       ///< Motor driver input pin for motor B direction control.
#define PWMB_RIGHT 6                  ///< PWM pin for motor B (right motor) speed control.
#define STBY_PIN 8                    ///< Standby pin for motor driver (shared between motors A and B).
#define ENCODER_LEFT_A_PIN 2          ///< Digital pin for left encoder signal A.
#define ENCODER_RIGHT_A_PIN 4         ///< Digital pin for right encoder signal A.
#define IR_SEND_PIN 9                 ///< IR transmitter pin for sending signals.
#define LEFT_RECEIVE_PIN A0           ///< Analog pin for receiving IR signals on the left side.
#define RIGHT_RECEIVE_PIN A1          ///< Analog pin for receiving IR signals on the right side.
#define KEY_MODE 10                   ///< Pin for detecting the mode selection key press.