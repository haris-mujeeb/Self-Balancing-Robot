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


class motion_controller {
  private:
    //Setting PID parameters
    double kp_balance = 55, kd_balance = 0.75; //  PID parameters for balance control.
    double kp_position = 10, ki_position = 0.26; // PID parameters for controlling speed.
    double kp_turn = 2.5, kd_turn = 0.5; // PID parameters for controlling the robot's turning.
    float angle_zero = 0, angular_velocity_zero = 0;

   // Setting Kalman filter parameters
   float dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, 
          R_angle = 0.5, C_0 = 1, K_comp_filter = 0.05;

  // Double data = 0;
    int16_t encoder_left_pulse_num_speed = 0;
    int16_t encoder_right_pulse_num_speed = 0;

  public:
    TB6612FNG motor;
    PIDController pid;
    mpu6050_base mpu;
    KalmanFilter kfilter;
    
    motion_controller() : motor(TB6612FNG(STBY_PIN, AIN1, BIN1, PWMA_LEFT, PWMB_RIGHT)),
      pid(PIDController(kp_balance, 0, kd_balance, -3000.0f, 3000.0f)),
      mpu(mpu6050_base()), 
      kfilter(KalmanFilter(dt, Q_angle, Q_gyro, R_angle, C_0)){
      };
      
    void init();
    //MsTimer2::set(5, balance);  // run balance every 5ms
    static void balance();   // void balance(float speed, float turn);
    void moveForward(float speed);
    void turnLeft(float rotation);
    void turnRight(float rotation);
    void moveBack(float speed);
};
