#include "motion_controller.hpp"
#include "EnableInterrupt.hpp"

#if (DEBUG_CONTROL) || (DEBUG_PID_PITCH) || (DEBUG_PID_YAW) || (DEBUG_PID_POSITION)
    String debugMsg = "[Debug]";
#endif

#if (PLOT_MODE)
    String plotMsg = "";
#endif

#if DEBUG_ENCODER
    String debugEncoderMsg = "[Debug]";
#endif

// Constants
constexpr uint8_t MINIMUM_ALLOWED_VOLTAGE = 6.0;   // Minimum allowed voltage for operation 
constexpr uint8_t VOLTAGE_MEASUREMENT_PERIOD = 100;   // Minimum allowed voltage for operation 
constexpr uint8_t POSITION_CONTROL_FREQUENCY = 8;   // e.g. one time after 8 interrupt calls to balance()

// Control Gains
constexpr double kp_balance = 55.0;
constexpr double kd_balance = 0.75;
constexpr double kp_position = 10*POSITION_CONTROL_FREQUENCY/8;
constexpr double kd_position = 0;
constexpr double ki_position = 0.26*POSITION_CONTROL_FREQUENCY/8;
constexpr double kp_turn = 2.5;
constexpr double kd_turn = 0.5;

// Kalman Filter and Complementary Filter Constants
constexpr float angle_zero = 0.0f;           ///< Default zero angle for the system
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
double setting_car_speed = 0;                ///< Desired speed of the robot
double setting_car_rotation_speed = 0;       ///< Desired rotational speed (yaw)
float pitch_angle = 0;                       ///< Measured pitch angle
float gyro_x = 0;                            ///< Gyro X-axis angular velocity
float gyro_z = 0;                            ///< Gyro Z-axis angular velocity
float voltage_value = 0;                     ///< Measured battery voltage

// Instances
motion_controller* controller_instance = nullptr; ///< Pointer to the motion controller instance for managing balance function
TB6612FNG motors(STBY_PIN, AIN1, BIN1, PWMA_LEFT, PWMB_RIGHT); ///< Motor controller instance for controlling the motors
mpu6050_base mpu;                               ///< MPU6050 sensor instance for motion sensing
KalmanFilter kalman(dt, Q_angle, Q_gyro, R_angle, C_0); ///< Kalman filter instance for angle estimation

// Function Declarations for controlling and updating robot state
void checkVoltageLevel(unsigned long&);
void updateSensorValues(float&, float&, float&);
void runPitchControl();
void updateEncoderValues();
void runPositionControl();
void runYawControl();
void updateMotorVelocities();

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
void motion_controller::init(){
  controller_instance = this;
  analogReference(INTERNAL);  ///< Use internal voltage reference for ADC
  motors.init();               ///< Initialize motor controller
  mpu.init();                  ///< Initialize MPU sensor

  // Enable encoder interrupts for left and right encoder
  enableInterrupt(ENCODER_LEFT_A_PIN | PINCHANGEINTERRUPT, encoderCounterLeftA, CHANGE);
  enableInterrupt(ENCODER_RIGHT_A_PIN, encoderCounterRightA, CHANGE);

  // Set up a timer to call the balance function at regular intervals
  MsTimer2::set(dt * 1000, balance);
  MsTimer2::start();

  DEBUG_PRINT(DEBUG_MODE, "[Debug] motion_controller setup complete.");
}

/**
 * @brief Balance control function executed periodically by the timer.
 * 
 * This function is responsible for maintaining the robot's balance. It checks the 
 * voltage level, reads sensor data, computes control outputs for pitch, position, 
 * and yaw, and applies appropriate motor speeds.
 */
void motion_controller::balance() {
  static unsigned long lastVoltageTime = 0; 

  // stop execution if voltage is low.
  checkVoltageLevel(lastVoltageTime);
  if(voltage_value <= MINIMUM_ALLOWED_VOLTAGE) {
    ERROR_PRINT("[Error] Voltage low!");  ///< Print error if voltage is below the minimum threshold
    motors.stop();  ///< Stop the motors if voltage is insufficient
    return;
  }

  sei();  // Enable global interrupts for sensor updates
  updateSensorValues(pitch_angle, gyro_x, gyro_z);  // Update sensor reading
  runPitchControl();     // Compute pitch control
  updateEncoderValues(); // Update encoder data

  // Update position and yaw control periodically
  position_control_period_count ++;
  if (position_control_period_count >= POSITION_CONTROL_FREQUENCY) {
    position_control_period_count = 0;
    runPositionControl(); ///< Compute position control output
    runYawControl();      ///< Compute yaw control output
  }

  // Compute PWM values for motor control
  pwm_left = pitch_pid_output - yaw_pid_output - position_pid_output;
  pwm_right = pitch_pid_output + yaw_pid_output - position_pid_output;
  
  // Update motor velocities with computed PWM values
  updateMotorVelocities();

  // Debugging and plotting sections
#if DEBUG_CONTROL 
 debugMsg += "[pwn_left:" + String(pwm_left) + "]"; 
 debugMsg += "[pwn_right:" + String(pwm_right) + "]"; 
#endif

#if DEBUG_PID_YAW
  debugMsg += "[YAW PID: " + String(yaw_pid_output) + "]";
#endif

#if DEBUG_PID_POSITION
  debugMsg += "[POS PID:" + String(position_pid_output) + "]";
  debugMsg += "[postion:" + String(robot_position) + "]";
#endif

#if (DEBUG_CONTROL) || defined(DEBUG_PID_PITCH) || defined(DEBUG_PID_YAW) || defined(DEBUG_PID_POSITION)
  DEBUG_PRINT(DEBUG_MODE, debugMsg);
  debugMsg = "[Debug]";
#endif

#if (PLOT_MODE)
  plotMsg += String(pitch_angle) + ","; 
  plotMsg += String(gyro_z)  + ","; 
  plotMsg += String(robot_position) + ","; 
  plotMsg += String(pitch_pid_output) + ","; 
  plotMsg += String(yaw_pid_output) + ","; 
  plotMsg += String(position_pid_output) + ","; 
  plotMsg += String(pwm_left) + ","; 
  plotMsg += String(pwm_right); 
  SEND_FOR_PLOT(true, plotMsg);
  plotMsg = "";
#endif

#if DEBUG_ENCODER
  // Debug encoder position data
  debugEncoderMsg += "[encoder_l_pos:" + String(encoder_left_position) 
    +"][encoder_r_pos:"+ String(encoder_right_position) + "]";
  DEBUG_PRINT(DEBUG_ENCODER, debugEncoderMsg)
  debugEncoderMsg = "[Debug]";
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
 */inline void checkVoltageLevel(unsigned long& voltage_measure_time) {
  if (millis() - voltage_measure_time >= VOLTAGE_MEASUREMENT_PERIOD) { // Call voltage_read() every 100ms
    voltage_measure_time = millis();
    voltage_value = (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5);
    debugMsg +=  "[voltage:" + String(voltage_value)+"]";
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
  mpu.read_mpu_6050_data();

  // Compute angle and angular velocities
  pitch_angle = atan2(mpu.acc_y, mpu.acc_z) * 57.3;
  gyro_x = (mpu.gyro_x - 128.1) / 131.0;
  gyro_z = -mpu.gyro_z / 131.0;

  // Update Kalman filter
  kalman.getAngle(pitch_angle, gyro_x);
}


/**
 * @brief Executes the pitch control algorithm.
 * 
 * This function computes the pitch PID output based on the balance control gains, 
 * the estimated angle from the Kalman filter, and the angular velocity.
 */
inline void runPitchControl() {
  // Compute balance control output
  pitch_pid_output = kp_balance * (kalman.angle - angle_zero) 
                    + kd_balance * (gyro_x  - angular_velocity_zero);
  Serial.print(kalman.angle);
  Serial.println(gyro_x);
}


/**
 * @brief Executes the yaw control algorithm.
 * 
 * Calculates the yaw PID output to control the rotational motion of the robot 
 * using the z-axis angular velocity and desired rotation speed.
 */
inline void runYawControl(){
  yaw_pid_output = setting_car_rotation_speed + kd_turn * gyro_z;
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
  robot_position += -setting_car_speed;
  robot_position = constrain(robot_position, -3000, 3000);
  position_pid_output = - kp_position * encoder_speed_filtered - ki_position * robot_position;
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