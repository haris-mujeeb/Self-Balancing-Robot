#include "motion_controller.hpp"
#include "EnableInterrupt.hpp"
#include "voltage.hpp"

#if (DEBUG_CONTROL) || (DEBUG_PID_PITCH) || (DEBUG_PID_YAW) || (DEBUG_PID_POSITION)
    String debugMsg = "";
#endif

#if (PLOT_MODE)
    String plotMsg = "";
#endif

#ifdef DEBUG_ENCODER
    String debugEncoderMsg = "";
#endif


/* Note: 
    'volatile' tells the compiler that the value of encoder_count_right_a can change at any time, outside of the normal program flow. 
    This is commonly used for variables that are modified within an interrupt service routine (ISR) or by another thread. */
// Volatile variables that track the pulse counts from the encoders.
volatile int long encoder_count_left_a = 0; 
volatile int long encoder_count_right_a = 0;
volatile double pwm_left = 0;
volatile double pwm_right = 0;
volatile int16_t encoder_left_position = 0;
volatile int16_t encoder_right_position = 0;
volatile uint8_t position_control_period_count = 0;
// volatile double left_position_pid_output = 0;
// volatile double right_position_pid_output = 0;
volatile double pitch_pid_output = 0;
volatile double position_pid_output = 0;
volatile double yaw_pid_output = 0;
// volatile double motor_position = 0;
volatile double encoder_speed_filtered = 0;
volatile double encoder_speed_filtered_old = 0;
volatile double robot_position = 0;
volatile double setting_car_speed = 0;
volatile double setting_car_rotation_speed = 0;
volatile double pitch_angle = 0; 
volatile double gyro_x = 0;
volatile double gyro_z = 0;
motion_controller* controller_instance = nullptr; // for attaching balance() to MsTimer2 

void encoderCounterLeftA() { encoder_count_left_a++;}
void encoderCounterRightA() { encoder_count_right_a++;}


void motion_controller::run(){
  controller_instance = this;
  motors.init();
  mpu.init();
  voltage_init();
  enableInterrupt(ENCODER_LEFT_A_PIN|PINCHANGEINTERRUPT, encoderCounterLeftA, CHANGE);
  enableInterrupt(ENCODER_RIGHT_A_PIN, encoderCounterRightA, CHANGE);
}


void motion_controller::run(){
  MsTimer2::set(dt*1000, balance);
  MsTimer2::start();
  DEBUG_PRINT(DEBUG_MODE, "motion_controller setup complete.");
}

void motion_controller::moveForward(float speed){
  setting_car_speed = speed;
  setting_car_rotation_speed = 0;
}

void motion_controller::moveBack(float speed){
  setting_car_speed = -speed;
  setting_car_rotation_speed = 0;
}

void motion_controller::turnLeft(float rotation_speed){
  setting_car_speed = 0;
  setting_car_rotation_speed = rotation_speed;
}

void motion_controller::turnRight(float rotation_speed){
  setting_car_speed = 0;
  setting_car_rotation_speed = -rotation_speed;
}

void motion_controller::stop(){
  setting_car_speed = 0;
  setting_car_rotation_speed = 0;
}

void motion_controller::balance() {
  static unsigned long lastVoltageTime = 0; 

  // stop execution if controller_instance not found.
  if (!controller_instance) { 
    ERROR_PRINT("no controller_instance found!");
    // stop motors and exit funciton
    return;
  }

  // stop execution if voltage is low.
  controller_instance->checkVoltageLevel(lastVoltageTime);
  if(last_voltage_value <= MINIMUM_ALLOWED_VOLTAGE) {
    // debug message
  #ifdef DEBUG_CONTROL
    DEBUG_PRINT(DEBUG_CONTROL ,"Voltage low!");
  #endif
    // stop motors and the exit funciton
    controller_instance->motors.stop();
    return;
  }

  // Enable global interrupts
  sei();

  
  // Read sensor data and 
  controller_instance->updateSensorValues();

  // compute pitch angle pid control output
  controller_instance->runPitchControl();

  // update total steps taken
  controller_instance->updateEncoderValues();

  // run position control after every 8 interrupt calls 
  position_control_period_count ++;
  if (position_control_period_count >= POSITION_CONTROL_FREQUENCY) {
    position_control_period_count = 0;
    
    // compute position pid control output
    controller_instance->runPositionControl();

    // compute yaw angle pid control output
    controller_instance->runYawControl();
    
    // Reset total distance count

}
  // update total pid output
  pwm_left = pitch_pid_output - yaw_pid_output - position_pid_output;
  pwm_right = pitch_pid_output + yaw_pid_output - position_pid_output;
  
  // Apply the control output to the motors
  controller_instance->updateMotorVelocities();

#if (DEBUG_CONTROL) 
 debugMsg += "[pwn_left:" + String(pwm_left) + "]"; 
 debugMsg += "[pwn_right:" + String(pwm_right) + "]"; 
#endif

// Print Debug messages
#ifdef DEBUG_PID_PITCH
  debugMsg += "[PITCH PID: " + String(pitch_pid_output) + "]";
#endif

#ifdef DEBUG_PID_YAW
  debugMsg += "[YAW PID: " + String(yaw_pid_output) + "]";
#endif

#ifdef DEBUG_PID_POSITION
  debugMsg += "[POS PID:" + String(position_pid_output) + "]";
  debugMsg += "[postion:" + String(robot_position) + "]";
#endif

#if (DEBUG_CONTROL) || (DEBUG_PID_PITCH) || (DEBUG_PID_YAW) || (DEBUG_PID_POSITION)
  DEBUG_PRINT(DEBUG_MODE, debugMsg);
  debugMsg = "";
#endif

#if (PLOT_MODE)
  plotMsg += String(controller_instance->kfilter.angle) + ","; 
  plotMsg += String(gyro_z) + ","; 
  plotMsg += String(robot_position) + ","; 
  plotMsg += String(pitch_pid_output) + ","; 
  plotMsg += String(yaw_pid_output) + ","; 
  plotMsg += String(position_pid_output) + ","; 
  plotMsg += String(pwm_left) + ","; 
  plotMsg += String(pwm_right); 
  SEND_FOR_PLOT(true, plotMsg);
  plotMsg = "";
#endif

#ifdef DEBUG_ENCODER
  debugEncoderMsg += "[encoder_l_pos:" + String(encoder_left_position) 
    +"][encoder_r_pos:"+ String(encoder_right_position) + "]";
  DEBUG_PRINT(DEBUG_ENCODER, debugEncoderMsg)
  debugEncoderMsg = "";
#endif
}

void motion_controller::checkVoltageLevel(unsigned long& lastVoltageTime) {
  if (millis() - lastVoltageTime >= 1000) { // Call voltage_read() every 100ms
    lastVoltageTime = millis();
    voltage_read();
  }
}

void motion_controller::updateEncoderValues(){
  encoder_left_position +=  pwm_left < 0 ? -encoder_count_left_a : encoder_count_left_a;
  encoder_right_position += pwm_right < 0 ? -encoder_count_right_a : encoder_count_right_a;
  encoder_count_left_a = 0;
  encoder_count_right_a = 0;
}

void motion_controller::updateSensorValues(){
controller_instance->mpu.read_mpu_6050_data(); 
  // update imu data

  // Compute angle and angular velocities
  pitch_angle = atan2(controller_instance->mpu.acc_y, controller_instance->mpu.acc_z) * 57.3;
  gyro_x = (controller_instance->mpu.gyro_x - 128.1) / 131.0;
  gyro_z = -controller_instance->mpu.gyro_z / 131.0;

  // Update Kalman filter
  controller_instance->kfilter.getAngle(pitch_angle, gyro_x);
}

void motion_controller::runPitchControl() {
  // Compute balance control output
  pitch_pid_output = kp_balance * (controller_instance->kfilter.angle - angle_zero) 
                    + kd_balance * (controller_instance->mpu.gyro_x  - angular_velocity_zero);
}

void motion_controller::runYawControl(){
  yaw_pid_output = setting_car_rotation_speed + kd_turn * gyro_z;
}

void motion_controller::runPositionControl(){
  double encoder_speed = (encoder_left_position + encoder_right_position) * 0.5;
  encoder_left_position = 0;
  encoder_right_position = 0;
  encoder_speed_filtered = encoder_speed_filtered_old * 0.7 + encoder_speed * 0.3; 
  encoder_speed_filtered_old = encoder_speed_filtered;
  robot_position += encoder_speed_filtered;
  robot_position += -setting_car_speed;
  robot_position = constrain(robot_position, -3000, 3000);
  position_pid_output = - kp_position * encoder_speed_filtered - ki_position * robot_position;
}

void motion_controller::updateMotorVelocities() {
  // Set motor A
  controller_instance->motors.motorA(pwm_left);

  // Set motor B
  controller_instance->motors.motorB(pwm_right);
}