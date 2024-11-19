#include "motion_controller.hpp"
#include "EnableInterrupt.hpp"

// Volatile variables that track the pulse counts from the encoders.
volatile int long encoder_count_left_a = 0; 
volatile int long encoder_count_right_a = 0;
volatile double pwm_left = 0;
volatile double pwm_right = 0;
/* Note: 
    'volatile' tells the compiler that the value of encoder_count_right_a can change at any time, outside of the normal program flow. 
    This is commonly used for variables that are modified within an interrupt service routine (ISR) or by another thread. */

void encoderCounterLeftA() {
  encoder_count_left_a += (pwm_left > 0) - (pwm_left < 0);   // Note: This logic eliminates branching, making it faster for embedded systems
}

void encoderCounterRightA() {
  encoder_count_right_a++;
}


void motion_controller::init(){
  motor.init();
  mpu.init();
  enableInterrupt(ENCODER_LEFT_A_PIN|PINCHANGEINTERRUPT, encoderCounterLeftA, CHANGE);
  enableInterrupt(ENCODER_RIGHT_A_PIN, encoderCounterRightA, CHANGE);
  //MsTimer2::set(5, balance);
  DEBUG_PRINT(DEBUG_MODE, "[Debug] motion_controller setup complete.");
}


void motion_controller::balance() {
  sei(); // set enable interrupts

  //  // PID with complimentary filtering:
  // mpu.calculate();
  // pid.Compute(0.0f, mpu.angle_pitch);
  // pwm_left = pid.Output;
  // pwm_right = pid.Output;
  // pwm_left = constrain(pwm_left, -255, 255);
  // pwm_right = constrain(pwm_right, -255, 255);
  // (pwm_left < 0) ? motor.motorA(-pwm_left, 1) : motor.motorA(pwm_left, 0);
  // (pwm_right < 0) ? motor.motorB(-pwm_right, 1) : motor.motorB(pwm_right, 0);

  // Kalman filter with PD controller
  mpu.read_mpu_6050_data();
  float angle_m = atan2(mpu.acc_y , mpu.acc_z) * 57.3;
  float gyro_x = (mpu.gyro_x - 128.1)/131;
  kfilter.getAngle(angle_m, gyro_x);
  float gyro_z = -mpu.gyro_z / 131;
  DEBUG_PRINT(DEBUG_KALMAN, "[Debug] [Kalman values] [angle_m: " + String(angle_m) + "] [gyro_x: " 
    + String(gyro_x) + "] [gyro_z: " + String(gyro_z)  + "]");
  double balance_control_output = kp_balance * (kfilter.angle - angle_zero) 
    + kd_balance * (gyro_x - angular_velocity_zero); 

  pwm_left = balance_control_output;
  pwm_right = balance_control_output;
  pwm_left = constrain(pwm_left, -255, 255);
  pwm_right = constrain(pwm_right, -255, 255);

  String debugMsg = "[Debug] [pwm_left: "+ String(pwm_left) + "]" 
    + " [pwm_right: "+ String(pwm_right) + "]"; 
  DEBUG_PRINT(DEBUG_MODE, debugMsg);
  (pwm_left > 0) ? motor.motorA(-pwm_left, 1) : motor.motorA(pwm_left, 0);
  (pwm_right > 0) ? motor.motorB(-pwm_right, 1) : motor.motorB(pwm_right, 0);
}