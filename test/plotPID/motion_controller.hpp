#pragma once
#include <stdint.h>
#include "HardwareSerial.h"
#include "debugConfig.h"
#include "pins.hpp"
#include "pid.hpp"
#include "tb6612fng.hpp"
#include "mpu6050_base.hpp"
#include "kalman_filter.hpp"
#include "MsTimer2.h"

constexpr uint8_t MINIMUM_ALLOWED_VOLTAGE = 6.0;   // Minimum allowed voltage for operation 
constexpr uint8_t POSITION_CONTROL_FREQUENCY = 8; // e.g. one time after 8 interrupt calls to balance()
constexpr double kp_balance = 55.0;           // PID parameter for balance control
constexpr double kd_balance = 0.75;          // PID parameter for balance control
constexpr double kp_position = 10*POSITION_CONTROL_FREQUENCY/8;            // PID parameter for speed control
constexpr double kd_position = 0;            // PID parameter for speed control
constexpr double ki_position = 0.26*POSITION_CONTROL_FREQUENCY/8;            // PID parameter for speed control
constexpr double kp_turn = 2.5;              // PID parameter for turning control
constexpr double kd_turn = 0.5;              // PID parameter for turning control
constexpr float angle_zero = 0.0f;           // Default angle zero
constexpr float angular_velocity_zero = 0.0f; // Default angular velocity zero
constexpr float dt = 0.007f;                 // Time step for control loop
constexpr float Q_angle = 0.001f;            // Process noise covariance for angle
constexpr float Q_gyro = 0.005f;             // Process noise covariance for gyro
constexpr float R_angle = 0.5f;              // Measurement noise covariance for angle
constexpr float C_0 = 1.0f;                  // Kalman filter constant
constexpr float K_comp_filter = 0.05f;       // Complementary filter constant

class motion_controller {
  public:
    TB6612FNG motors;
    mpu6050_base mpu;
    KalmanFilter kfilter;
    
    motion_controller(); 
    void run();
    static void balance();   // void balance(float speed, float turn);
    void moveForward(float speed);
    void moveBack(float speed);
    void turnLeft(float rotation_speed);
    void turnRight(float rotation_speed);
    void stop();

  private:
    void checkVoltageLevel(unsigned long& lastVoltageTime);
    void updateSensorValues();
    void updateEncoderValues();
    void runPitchControl();
    void runYawControl();
    void runPositionControl();
    void updateMotorVelocities();

};
