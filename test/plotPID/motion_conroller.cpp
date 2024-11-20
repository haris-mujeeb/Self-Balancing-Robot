#include "motion_controller.hpp"
#include "EnableInterrupt.hpp"
#include "voltage.hpp"


// Volatile variables that track the pulse counts from the encoders.
volatile int long encoder_count_left_a = 0; 
volatile int long encoder_count_right_a = 0;
volatile uint8_t pwm_left = 0;
volatile uint8_t pwm_right = 0;
/* Note: `
    'volatile' tells the compiler that the value of encoder_count_right_a can change at any time, outside of the normal program flow. 
    This is commonly used for variables that are modified within an interrupt service routine (ISR) or by another thread. */

motion_controller* controller_instance = nullptr; // for attaching balance() to MsTimer2 


void encoderCounterLeftA() {
  encoder_count_left_a += (pwm_left > 0) - (pwm_left < 0);   // Note: This logic eliminates branching, making it faster for embedded systems
}


void encoderCounterRightA() {
  encoder_count_right_a++;
}


void motion_controller::init(){
  controller_instance = this;
  motor.init();
  mpu.init();
  voltage_init();
  enableInterrupt(ENCODER_LEFT_A_PIN|PINCHANGEINTERRUPT, encoderCounterLeftA, CHANGE);
  enableInterrupt(ENCODER_RIGHT_A_PIN, encoderCounterRightA, CHANGE);
  MsTimer2::set(dt*1000, balance);
  MsTimer2::start();
  DEBUG_PRINT(DEBUG_MODE, "[Debug] motion_controller setup complete.");
}


void motion_controller::balance() {
  static unsigned long lastVoltageTime = 0; 
  if (millis() - lastVoltageTime >= 1000) { // Call voltage_read() every 1000ms
    lastVoltageTime = millis();
    voltage_read();
  }

  if(controller_instance && (last_value > 6.0)) {
    // set enable interrupts
    sei(); 
    // Kalman filter with PD controller
    controller_instance->mpu.read_mpu_6050_data();
    float angle_m = atan2(controller_instance->mpu.acc_y , controller_instance->mpu.acc_z) * 57.3;
    float gyro_x = (controller_instance->mpu.gyro_x - 128.1)/131;
    controller_instance->kfilter.getAngle(angle_m, gyro_x);
    float gyro_z = -controller_instance->mpu.gyro_z / 131;
    DEBUG_PRINT(DEBUG_KALMAN, "[Debug] [Kalman values] [angle_m: " + String(angle_m) + "] [gyro_x: " 
      + String(gyro_x) + "] [gyro_z: " + String(gyro_z)  + "]");
    double balance_control_output = controller_instance->kp_balance * (controller_instance->kfilter.angle - controller_instance->angle_zero) 
      + controller_instance->kd_balance * (gyro_x - controller_instance->angular_velocity_zero) ; 

    int16_t pwm_signed_value = constrain((int16_t)balance_control_output, -255, 255);
    pwm_left = (uint8_t)abs(pwm_signed_value);
    pwm_right = (uint8_t)abs(pwm_signed_value);

    String debugMsg = String(controller_instance->kfilter.angle) + "," + String(balance_control_output)
      + "," + String(pwm_left);
    DEBUG_PRINT(DEBUG_CONTROL, debugMsg);
    controller_instance->motor.motorA(pwm_left, pwm_signed_value > 0);
    controller_instance->motor.motorB(pwm_right, pwm_signed_value > 0);
  } else {
        DEBUG_PRINT(DEBUG_CONTROL, "[Debug] balance() not called.");
  }
}