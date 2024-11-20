#include "motion_controller.hpp"
#include "EnableInterrupt.hpp"
#include "voltage.hpp"

#ifdef DEBUG_CONTROL
    String debugMsg = "";
#endif

/* Note: 
    'volatile' tells the compiler that the value of encoder_count_right_a can change at any time, outside of the normal program flow. 
    This is commonly used for variables that are modified within an interrupt service routine (ISR) or by another thread. */
// Volatile variables that track the pulse counts from the encoders.
volatile int long encoder_count_left_a = 0; 
volatile int long encoder_count_right_a = 0;
volatile int16_t pwm_left = 0;
volatile int16_t pwm_right = 0;
volatile int16_t encoder_left_pulse_num_speed = 0;
volatile int16_t encoder_right_pulse_num_speed = 0;

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

  // stop execution if controller_instance not found.
  if (!controller_instance) { 
    ERROR_PRINT("[Error] no controller_instance found!");
    // stop motors and exit funciton
    return;
  }

  // stop execution if voltage is low.
  controller_instance->checkVoltageLevel(lastVoltageTime);
  if(last_value <= MINIMUM_ALLOWED_VOLTAGE) {
    // debug message
  #ifdef DEBUG_CONTROL
    DEBUG_PRINT(DEBUG_CONTROL ,"[Debug] Voltage low!");
  #endif
    // stop motors and the exit funciton
    controller_instance->motor.stop();
    return;
  }

  // Enable global interrupts
  sei();

  float angle_m = 0, gyro_x = 0, gyro_z = 0;
  
  // Read sensor data and 
  controller_instance->updateSensorValues(angle_m, gyro_x, gyro_z);

  // compute pitch angle pid control output
  controller_instance->runPitchControl(gyro_x);

  // compute yaw angle pid control output
  controller_instance->runYawControl();

  // compute position pid control output
  controller_instance->runPositionControl();

  // Apply the control output to the motors
  controller_instance->updateMotorVelocities();

  // Debug outputs
#ifdef DEBUG_CONTROL
  DEBUG_PRINT(DEBUG_CONTROL, debugMsg);
#endif
}

void motion_controller::checkVoltageLevel(unsigned long& lastVoltageTime) {
  if (millis() - lastVoltageTime >= 100) { // Call voltage_read() every 100ms
    lastVoltageTime = millis();
    voltage_read();
  }
}

void motion_controller::updateSensorValues(float& angle_m, float& gyro_x, float& gyro_z){
    controller_instance->mpu.read_mpu_6050_data(); // update imu data

  // Compute angle and angular velocities
  angle_m = atan2(controller_instance->mpu.acc_y, controller_instance->mpu.acc_z) * 57.3;
  gyro_x = (controller_instance->mpu.gyro_x - 128.1) / 131;
  gyro_z = -controller_instance->mpu.gyro_z / 131;

  // Update Kalman filter
  controller_instance->kfilter.getAngle(angle_m, gyro_x);
}

void motion_controller::runPitchControl(float& gyro_x) {
  // Compute balance control output
  double pitch_pid_output = kp_balance * (controller_instance->kfilter.angle - angle_zero) 
                                + kd_balance * (gyro_x - angular_velocity_zero);

  // Constrain control output and update motor PWM values

  pwm_left = pitch_pid_output;
  pwm_right = pitch_pid_output;

  // Debug output for PID results
  debugMsg = "[Debug] [PITCH PID: " + String(pitch_pid_output) + "]";
}

void motion_controller::runYawControl(){
  double yaw_pid_output = 0;

#ifdef DEBUG_PID_YAW
  // Debug output for PID results
  debugMsg += "[Debug] [YAW PID: " + String(yaw_pid_output) + "]";
#endif
}

void motion_controller::runPositionControl(){
  double position_pid_output = 0;

#ifdef DEBUG_PID_POSITION
    // Debug output for PID results
  debugMsg += "[POS PID: " + String(position_pid_output) + "]";
#endif
}

void motion_controller::updateMotorVelocities() {
  // Set motor A
  controller_instance->motor.motorA(pwm_left);

  // Set motor B
  controller_instance->motor.motorB(pwm_right);
}