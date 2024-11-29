#include "motionController.hpp"
#include "EnableInterrupt.hpp"
#include "AccelStepper.h"

#if DEBUG_MODE
  String debugMsg = "";
#if DEBUG_ENCODER
  String debugEncoderMsg = "";
#endif
#endif

#if (PLOT_MODE)
    String plotMsg = "";
#endif


// Constants
constexpr uint8_t MINIMUM_ALLOWED_VOLTAGE = 5.0;   // Minimum allowed voltage for operation 
constexpr uint16_t VOLTAGE_MEASUREMENT_PERIOD = 200;   // Minimum allowed voltage for operation 
constexpr uint8_t POSITION_CONTROL_FREQUENCY = 8;   // e.g. one time after 8 interrupt calls to balance()
constexpr uint16_t START_UP_TIME = 5000;  // Time for robot to stand upright.
constexpr float ENCODER_STEP_PER_CM = 32.0;  // Time for robot to stand upright.

// Control Gains
constexpr double kp_balance = 35;
constexpr double kd_balance = 0.75;
constexpr double kd_position = 10*POSITION_CONTROL_FREQUENCY/8;
constexpr double kp_position = 0.26*POSITION_CONTROL_FREQUENCY/8;
constexpr double kp_turn = 5;
constexpr double kd_turn = 0.5;
// constexpr double ki_turn = 0.2;

// Kalman Filter and Complementary Filter Constants
// constexpr float angle_zero = 0.0f;           ///< Default zero angle for the system
constexpr float angular_velocity_zero = 0.0f; ///< Default zero angular velocity
constexpr float dt = 0.005f;                 ///< Time step for the control loop (in seconds)
constexpr float Q_angle = 0.001f;            ///< Process noise covariance for angle in the Kalman filter
constexpr float Q_gyro = 0.005f;             ///< Process noise covariance for gyro readings
constexpr float R_angle = 0.5f;              ///< Measurement noise covariance for angle measurements
constexpr float C_0 = 1.0f;                  ///< Kalman filter constant for angle measurement update
constexpr float K_comp_filter = 0.05f;       ///< Complementary filter constant for sensor fusion

// Volatile variables for encoder counts (due to interrupt handling)
volatile int long encoder_count_left_a = 0;  ///< Left encoder count (updated in ISR)
volatile int long encoder_count_right_a = 0; ///< Right encoder count (updated in ISR)
/* 
Note:
    'volatile' keyword ensures the compiler does not optimize these variables, as their values may change unexpectedly due to interrupts.
*/


double pwm_left = 0;                         ///< Left motor PWM (control value)
double pwm_right = 0;                        ///< Right motor PWM (control value)
int long encoder_left_position = 0;          ///< Position of the left encoder
int long encoder_right_position = 0;         ///< Position of the right encoder
unsigned int position_control_period_count = 0; ///< Counter for position control frequency
double pitch_pid_output = 0;                 ///< Output of the pitch PID controller
double position_pid_output = 0;              ///< Output of the position PID controller
double yaw_pid_output = 0;                  ///< Output of the yaw PID controller
double encoder_speed_filtered = 0;           ///< Filtered encoder speed value
double encoder_speed_filtered_old = 0;       ///< Previous filtered encoder speed value
double robot_position = 0;                   ///< Current position of the robot
double move_to_position = 0;                   ///< Temporary taget position for the robot
double final_position = 0;                   ///< Final position of the robot
double setting_car_speed = 0;                ///< Desired speed of the robot
double setting_car_rotation_speed = 0;       ///< Desired rotational speed (yaw)
float pitch_angle = 0;                       ///< Measured pitch angle
float angle_zero = 0.0f;                    ///< Default zero angle for the system
float yaw_angle = 0;                       ///< Measured pitch angle
float alpha = 0.95;                       ///< Measured pitch angle
double desired_yaw_angle = 0;                   ///< Current position of the robot
float gyro_x = 0;                            ///< Gyro X-axis angular velocity
float gyro_z = 0;                            ///< Gyro Z-axis angular velocity
float voltage_value = 0;                     ///< Measured battery voltage
bool disableWdt = true;
motionState currentRobotState;

// Instances
TB6612FNG motors(STBY_PIN, AIN1, BIN1, PWMA_LEFT, PWMB_RIGHT); ///< Motor controller instance for controlling the motors
mpu6050Base mpu;                               ///< MPU6050 sensor instance for motion sensing
KalmanFilter kalman(dt, Q_angle, Q_gyro, R_angle, C_0); ///< Kalman filter instance for angle estimation

// Function Declarations for controlling and updating robot state
void checkVoltageLevel(unsigned long&);
void updateSensorValues(float&, float&, float&);
void runPitchControl();
void updateEncoderValues();
void runPositionControl();
void runYawControl();
void updateMotorVelocities();
void checkStopConditions();

// Interrupt Handlers for Encoder Counts
void encoderCounterLeftA() { encoder_count_left_a++;} ///< ISR for left encoder count (incremented on pin change)
void encoderCounterRightA() { encoder_count_right_a++;} ///< ISR for right encoder count (incremented on pin change)


/**
 * @brief Initializes the motion controller system, setting up sensors and hardware.
 * 
 * This function performs the necessary setup tasks for the motion controller, including 
 * configuring the motors, initializing the MPU sensor, enabling interrupts for encoders,
 * and setting up a timer for the balance control function.
 */
void motionController::run(){
  analogReference(INTERNAL);  ///< Use internal voltage reference for ADC
  motors.init();               ///< Initialize motor controller
  mpu.init();                  ///< Initialize MPU sensor

  // Enable encoder interrupts for left and right encoder
  enableInterrupt(ENCODER_LEFT_A_PIN | PINCHANGEINTERRUPT, encoderCounterLeftA, CHANGE);
  enableInterrupt(ENCODER_RIGHT_A_PIN, encoderCounterRightA, CHANGE);
  
  // Configure watchdog timer
  wdt_enable(WDTO_500MS); // Watchdog timeout set to 500 milli seconds
  disableWdt = false;
  DEBUG_PRINT(DEBUG_WATCHDOG, "WatchDog enabled!")

  // Set up a timer to call the balance function at regular intervals
  currentRobotState = BALANCE;
  MsTimer2::set(dt * 1000, balance);
  // while(digitalRead(KEY_MODE)){}; // stop execution until the push button is pressed.
  MsTimer2::start();

  DEBUG_PRINT(DEBUG_MODE, "motion_controller setup complete.");
}

/**
 * @brief Balance control function executed periodically by the timer.
 * 
 * This function is responsible for maintaining the robot's balance. It checks the 
 * voltage level, reads sensor data, computes control outputs for pitch, position, 
 * and yaw, and applies appropriate motor speeds.
 */
void motionController::balance() {
  // stop execution if voltage is low.
  static unsigned long lastVoltageTime = 0;
  checkVoltageLevel(lastVoltageTime);
  if(voltage_value <= MINIMUM_ALLOWED_VOLTAGE) {
    ERROR_PRINT("Voltage low!");  ///< Print error if voltage is below the minimum threshold
    motors.stop();  ///< Stop the motors if voltage is insufficient
    return;
  }
 
  static unsigned long firstStartedBalancingTime = ;
  sei();  // Enable global interrupts
  updateSensorValues(pitch_angle, gyro_x, gyro_z);
  checkStopConditions();
  runPitchControl();  ///< Compute pitch control output
  updateEncoderValues();

  if(BALANCE) {
    if (millis() - firstStartedBalancingTime > 4000 
        && pitch_angle < 20) {
      currentRobotState = STANDBY;
      }
  } else{
    // Update position and yaw control periodically
    position_control_period_count ++;
    if (position_control_period_count >= POSITION_CONTROL_FREQUENCY && currentRobotState != ) {
      position_control_period_count = 0;
      runPositionControl(); ///< Compute position control output
      runYawControl();      ///< Compute yaw control output
    }
  }

  // Compute and update PWM values for motor control
  pwm_left = pitch_pid_output - yaw_pid_output - position_pid_output;
  pwm_right = pitch_pid_output + yaw_pid_output - position_pid_output;
  updateMotorVelocities();

  // Debugging and plotting sections
  #if DEBUG_CONTROL 
  debugMsg += "[pwn_left:" + String(pwm_left) + "]"; 
  debugMsg += "[pwn_right:" + String(pwm_right) + "]"; 
  #endif

  #if DEBUG_PID_YAW
    debugMsg += "[YAW PID: " + String(yaw_pid_output) + "]";
  #endif

  #if (DEBUG_PID_POSITION) && (DEBUG_MODE)
    debugMsg += "[POS PID:" + String(position_pid_output) + "]";
    debugMsg += "[postion:" + String(robot_position) + "]";
  #endif

  #if (DEBUG_MODE)
    DEBUG_PRINT(DEBUG_MODE, debugMsg);
    debugMsg = "";
  #endif

  #if (PLOT_MODE)
    plotMsg += String(encoder_speed_filtered) + ","; 
    plotMsg += String(encoder_speed_filtered_old)  + ","; 
    plotMsg += String(robot_position) + ","; 
    plotMsg += String(pitch_pid_output) + ","; 
    plotMsg += String(yaw_pid_output) + ","; 
    plotMsg += String(position_pid_output) + ","; 
    plotMsg += String(pwm_left) + ","; 
    plotMsg += String(pwm_right); 
    SEND_FOR_PLOT(plotMsg);
    plotMsg = "";
  #endif

  #if DEBUG_ENCODER
    // Debug encoder position data
    debugEncoderMsg += "[ENC L:" + String(encoder_left_position) 
      +"][ENC R:"+ String(encoder_right_position) + "]";
    DEBUG_PRINT(DEBUG_ENCODER, debugEncoderMsg)
    debugEncoderMsg = "";
  #endif
}


// Helper Function Definitions

/**
 * @brief Checks if the voltage level of the system is within the acceptable range.
 * 
 * This function reads the voltage from the specified pin and updates the voltage value. 
 * If the voltage level is below the minimum threshold, the system is stopped to prevent damage.
 * 
 * @param voltage_measure_time Reference to the last voltage measurement time in milliseconds.
 */
inline void checkVoltageLevel(unsigned long& voltage_measure_time) {
  if (millis() - voltage_measure_time >= VOLTAGE_MEASUREMENT_PERIOD) { // Call voltage_read() every 100ms
    voltage_measure_time = millis();
    voltage_value = (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5);
  #if DEBUG_VOLTAGE
    debugMsg +=  "[voltage:" + String(voltage_value)+"]";
  #endif
  }
}


/**
 * @brief Updates encoder position values based on the current encoder counts.
 * 
 * This function calculates the net encoder position by adding or subtracting the encoder counts 
 * based on the motor's direction. The counts are then reset for the next measurement cycle.
 */
inline void updateEncoderValues(){
  encoder_left_position +=  pwm_left < 0 ? -encoder_count_left_a : encoder_count_left_a;
  encoder_right_position += pwm_right < 0 ? -encoder_count_right_a : encoder_count_right_a;
  encoder_count_left_a = 0;
  encoder_count_right_a = 0;
}


/**
 * @brief Updates the sensor values using the MPU module.
 * 
 * Reads data from the MPU6050 sensor to compute the pitch angle and angular velocities. 
 * The calculated pitch angle is then processed through a Kalman filter for better estimation.
 * 
 * @param pitch_angle Reference to the pitch angle variable to be updated.
 * @param gyro_x Reference to the x-axis angular velocity variable to be updated.
 * @param gyro_z Reference to the z-axis angular velocity variable to be updated.
 */
inline void updateSensorValues(float& pitch_angle, float& gyro_x, float& gyro_z){
  // Read and compute angles and angular velocities
  mpu.read_mpu_6050_data();
  gyro_x = mpu.getGyroX();
  gyro_z = mpu.getGyroZ();
  pitch_angle = mpu.getPitchAngle();
  if (currentRobotState == BALANCE) {
    yaw_angle += mpu.getGyroZ() * dt;
  }
  
  // Update Kalman filter
  pitch_angle = kalman.getAngle(pitch_angle, gyro_x);
}


/**
 * @brief Executes the pitch control algorithm.
 * 
 * This function computes the pitch PID output based on the balance control gains, 
 * the estimated angle from the Kalman filter, and the angular velocity.
 */
inline void runPitchControl() {
  if (millis() - sinceStartUp)
  // Compute balance control output
  pitch_pid_output = kp_balance * (pitch_angle - constrain(angle_zero, -0.5, 0.5))
                    + kd_balance * (gyro_x  - angular_velocity_zero);
}


/**
 * @brief Executes the yaw control algorithm.
 * 
 * Calculates the yaw PID output to control the rotational motion of the robot 
 * using the z-axis angular velocity and desired rotation speed.
 */
inline void runYawControl(){
  if(currentRobotState == ROTATING){

    setting_car_rotation_speed = kp_turn * (yaw_angle - desired_yaw_angle) ;
    setting_car_rotation_speed = constrain(setting_car_rotation_speed, -50, 50);
    if (abs(yaw_angle - desired_yaw_angle) < 0.5) {
      currentRobotState = STANDBY;
    }
  } else {
    setting_car_rotation_speed = 0;
  }
  yaw_pid_output = setting_car_rotation_speed + kd_turn * gyro_z 
                  // + ki_turn * (yaw_angle - desired_yaw_angle)
                  ;
}


/**
 * @brief Executes the position control algorithm.
 * 
 * This function uses encoder readings to compute the position PID output, 
 * which helps in maintaining or changing the robot's position based on the target speed.
 */
inline void runPositionControl(){
  double encoder_speed = (encoder_left_position + encoder_right_position) * 0.5; 
  encoder_left_position = 0;
  encoder_right_position = 0;
  encoder_speed_filtered = encoder_speed_filtered_old * 0.7 + encoder_speed * 0.3; 
  encoder_speed_filtered_old = encoder_speed_filtered;
  robot_position += encoder_speed_filtered;
  // robot_position += -setting_car_speed;
  
  if(currentRobotState == MOVING){
    position_pid_output = - kp_position 
                            * (constrain(robot_position, -1000, 1000) 
                              - constrain(move_to_position, -50, 50)) 
                        - kd_position * encoder_speed_filtered;
    move_to_position = move_to_position*0.7 + final_position*0.3;
    angle_zero = position_pid_output/100;
    if (abs(robot_position - final_position) < 20) {
        currentRobotState = STANDBY;
      }
  } else {
    position_pid_output = 0;
  }

}


/**
 * @brief Updates motor velocities based on the calculated PWM values.
 * 
 * This function applies the computed PWM values to the motor controller to drive the motors.
 */
inline void updateMotorVelocities() {
  // Set motor A and B
  motors.motorA(pwm_left);
  motors.motorB(pwm_right);
}


/**
 * @brief Checks the stop conditions for the robot based on its pitch angle.
 * 
 * This function checks if the robot's pitch angle exceeds 20 degrees. If it does,
 * the motors are stopped, and the system enters an infinite loop. The watchdog timer
 * (WDT) will reset the system if it is not reset within the timeout period. If the 
 * angle is within acceptable limits, the WDT is reset to continue normal operation.
 * 
 * @note If the watchdog timer is disabled (via `disableWdt`), the WDT is not used.
 */
inline void checkStopConditions(){
  // Check if the absolute pitch angle exceeds 20 degrees and the watchdog timer (WDT) is enabled
  if(!disableWdt && abs(position_pid_output) > 1000) {
    /**
     * @brief Logs an error message and stops the motors if the pitch angle exceeds 20 degrees.
     * 
     * The system will stop the motors and enter an infinite loop. The WDT will reset the
     * system after the timeout period if the loop is not broken (i.e., the WDT is not reset).
     */
    ERROR_PRINT("Restarting...");
        
    // Stop the motors to prevent further movement
    motors.stop();

     /**
     * @brief Infinite loop to prevent further execution.
     * 
     * The system will stay in this loop until the WDT resets the system.
     */
    while (true) { 
    // The watchdog timer will cause a system reset if the WDT is not reset within the timeout period
    // The system will stay in this loop until it is reset by the watchdog     
    }

    // If the angle is within acceptable limits, reset the watchdog timer to keep the system running
    } else if(!disableWdt) {
    /**
     * @brief Resets the watchdog timer if the system is operating normally.
     * 
     * This ensures that the system will not be reset by the WDT if it is functioning properly.
     */
    wdt_reset();     // Reset the WDT to prevent a system reset
  }
}


/**
 * @brief Moves the robot forward at the specified speed.
 * 
 * Sets the forward speed for the robot while ensuring no rotational motion.
 * 
 * @param speed The forward speed for the robot (positive value).
 */
void motionController::moveForward(float speed){
  setting_car_speed = speed;
  setting_car_rotation_speed = 0;
}


/**
 * @brief Moves the robot backward at the specified speed.
 * 
 * Sets the backward speed for the robot while ensuring no rotational motion.
 * 
 * @param speed The backward speed for the robot (positive value).
 */
void motionController::moveBack(float speed){
  setting_car_speed = -speed;
  setting_car_rotation_speed = 0;
}


/**
 * @brief Turns the robot to the left at the specified rotation speed.
 * 
 * Stops forward/backward motion and sets the rotation speed for a left turn.
 * 
 * @param rotation_speed The angular speed for left turn (positive value).
 */
void motionController::turnLeft(float rotation_speed){
  setting_car_speed = 0;
  setting_car_rotation_speed = rotation_speed;
}


/**
 * @brief Turns the robot to the right at the specified rotation speed.
 * 
 * Stops forward/backward motion and sets the rotation speed for a right turn.
 * 
 * @param rotation_speed The angular speed for right turn (positive value).
 */
void motionController::turnRight(float rotation_speed){
  setting_car_speed = 0;
  setting_car_rotation_speed = -rotation_speed;
}

/**
 * @brief Moves the robot a specified distance in centimeters.
 * 
 * This function converts the given distance in centimeters to the corresponding 
 * encoder steps and stores it in the final_position variable.
 * The conversion uses the constant ENCODER_STEP_PER_CM, which defines the 
 * number of encoder steps per centimeter.
 * 
 * @param dist_in_cm The distance to move in centimeters.
 */
void motionController::moveCentimeters(float dist_in_cm) {
  final_position = dist_in_cm * ENCODER_STEP_PER_CM;
}


/**
 * @brief Sets the desired yaw angle to turn the robot by a specified number of degrees.
 * 
 * This function sets the desired yaw angle for the robot to rotate by the given
 * number of degrees in a clockwise direction. The angle is stored in the 
 * desired_yaw_angle variable.
 * 
 * @param degreesCW The angle in degrees to turn the robot clockwise.
 */
void motionController::turnDegrees(float degreesCW){
  currentRobotState = ROTATING;
  desired_yaw_angle = degreesCW;
}


/**
 * @brief Stops the robot's movement.
 * 
 * Sets both the forward/backward and rotation speeds to zero, halting all motion.
 */
void motionController::stop(){
  currentRobotState = STANDBY;
  setting_car_speed = 0;
  setting_car_rotation_speed = 0;
}


/**
 * @brief Safely retrieves the yaw angle and robot position.
 * 
 * This function disables interrupts temporarily to ensure atomic access
 * to `yaw_angle` and `robot_position`, which are updated in an interrupt context.
 * 
 * @param yaw Reference to store the retrieved yaw angle.
 * @param position Reference to store the retrieved robot position.
 */
void motionController::getRobotStateData(float& distance, float&yaw){
  cli(); // Disable interrupts
  distance = robot_position;
  yaw = yaw_angle;
  sei(); // Re-enable interrupts
}
