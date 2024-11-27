#pragma once

#include <stdint.h>
#include "HardwareSerial.h"
#include "debugConfig.h"
#include "pins.hpp"
#include "tb6612fng.hpp"
#include "mpu6050_base.hpp"
#include "kalman_filter.hpp"
#include "MsTimer2.h"
#include <avr/wdt.h>


/**
 * @class motion_controller
 * @brief A class to control the motion of a self-balancing robot.
 * 
 * This class provides methods to initialize the robot's control system and control its movement, 
 * including forward, backward, turning, and stopping. It leverages a Kalman filter for state estimation 
 * and uses PID controllers for stable motion.
 */
class motion_controller {
  public:
    /**
     * @brief Constructor for the motion_controller class.
     */
    motion_controller(){};
    
    /**
     * @brief Initializes the motion controller and its components.
     * 
     * Sets up sensors, motor drivers, interrupts, and the timer to manage the robot's balance control loop.
     */
    void run();

    /**
     * @brief Balance control loop for the robot.
     * 
     * This function is called periodically by a timer to maintain the robot's balance. 
     * It calculates control outputs based on sensor data and applies them to the motors.
     */
    static void balance();

    /**
     * @brief Moves the robot forward at the specified speed.
     * 
     * @param speed The forward speed for the robot (positive value).
     */
    void moveForward(float speed);

    /**
     * @brief Moves the robot backward at the specified speed.
     * 
     * @param speed The backward speed for the robot (positive value).
     */
    void moveBack(float speed);

    /**
     * @brief Turns the robot to the left at the specified rotation speed.
     * 
     * @param rotation_speed The angular speed for left turn (positive value).
     */
    void turnLeft(float rotation_speed);

    /**
     * @brief Turns the robot to the right at the specified rotation speed.
     * 
     * @param rotation_speed The angular speed for right turn (positive value).
     */
    void turnRight(float rotation_speed);


    void moveCentimeters(float dist_in_cm);

    void turnDegrees(float degreesCW);


    /**
     * @brief Stops the robot's movement.
     * 
     * Sets both the forward speed and rotation speed to zero, halting all motion.
     */
    void stop();
};
