#pragma once

#include <stdint.h>
#include "HardwareSerial.h"
#include "debugConfig.h"
#include "pins.hpp"
#include "tb6612fng.hpp"
#include "mpu6050Base.hpp"
#include "kalmanFilter.hpp"
#include "MsTimer2.h"
#include <avr/wdt.h>

enum motionState{ 
  STARTING,
  STANDBY,
  MOVING,
  TURNING,
  NUMBER_OF_MOTION_STATES
};


/**
 * @class motion_controller
 * @brief A class to control the motion of a self-balancing robot.
 * 
 * This class provides methods to initialize the robot's control system and control its movement, 
 * including forward, backward, turning, and stopping. It leverages a Kalman filter for state estimation 
 * and uses PID controllers for stable motion.
 */
class motionController {
  public:
    /**
     * @brief Constructor for the motion_controller class.
     */
    motionController(){};
    
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


    /**
     * @brief Moves the robot a specified distance in centimeters.
     * 
     * Converts the given distance in centimeters to encoder steps using the 
     * constant `ENCODER_STEP_PER_CM` and stores it in `final_position`.
     * 
     * @param dist_in_cm The distance to move in centimeters.
     */
    void moveCentimeters(float dist_in_cm);


    /**
     * @brief Sets the desired yaw angle to turn the robot by a specified number of degrees.
     * 
     * @param degreesCW The angle in degrees for clockwise rotation.
     */
    void turnDegrees(float degreesCW);


    /**
     * @brief Stops the robot's movement.
     * 
     * Sets both the forward speed and rotation speed to zero, halting all motion.
     */
    void stop();


  /**
   * @brief Safely retrieves the yaw angle and robot position.
   * 
   * This function disables interrupts temporarily to ensure atomic access
   * to `yaw_angle` and `robot_position`, which are updated in an interrupt context.
   * 
   * @param yaw Reference to store the retrieved yaw angle.
   * @param position Reference to store the retrieved robot position.
   */
    void getRobotStateData(float& distance, float&yaw);
};