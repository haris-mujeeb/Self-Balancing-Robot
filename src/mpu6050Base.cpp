#include "mpu6050Base.hpp"

// Calibration offsets for gyroscope axes
float gyro_x_cal = 0;                 ///< Calibration offset for gyroscope X axis
float gyro_y_cal = 0;                 ///< Calibration offset for gyroscope Y axis
float gyro_z_cal = 0;                 ///< Calibration offset for gyroscope Z axis

// Angle and sensor data variables
float angle_pitch = 0;               ///< Pitch angle calculated from sensor data
float angle_roll = 0;                ///< Roll angle calculated from sensor data
float acc_total_vector = 0;          ///< Total vector calculated from accelerometer readings
float angle_roll_acc = 0;            ///< Roll angle based on accelerometer data
float angle_pitch_acc = 0;           ///< Pitch angle based on accelerometer data
float angle_pitch_output = 0;        ///< Filtered pitch angle output
float angle_roll_output = 0;         ///< Filtered roll angle output

// Flag to indicate if gyro angles have been initialized
bool set_gyro_angles = false;        ///< Flag indicating whether gyro angles have been initialized

// Zero angle and velocity calibration values
// float angle_zero = 0;                ///< Zero angle for pitch axis
// float angular_velocity_zero = 0;     ///< Zero angular velocity for pitch axis


/**
 * @brief Initializes the MPU6050 sensor and calibrates the gyroscope.
 * 
 * This function sets up the MPU6050 registers, performs gyroscope calibration by 
 * averaging a set of samples, and calculates the gyro offsets for all axes.
 */
void mpu6050Base::init() {
    Wire.beginTransmission(mpu6050_addr);         // Start communication with MPU6050
  if (Wire.endTransmission() != 0) {  // Non-zero indicates an issue (no acknowledgment)
    ERROR_PRINT("MPU6050 not connected!");
    return;  // Exit the function as there's no MPU6050 to read from
  }

  setup_mpu_6050_registers();  // Set up MPU6050 registers for operation

  // Perform gyroscope calibration by taking multiple samples
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    read_mpu_6050_data();  // Read sensor data
    gyro_x_cal += gyro_x;  // Sum gyro X values for calibration
    gyro_y_cal += gyro_y;  // Sum gyro Y values for calibration
    gyro_z_cal += gyro_z;  // Sum gyro Z values for calibration
    delay(3);  // Wait for the next sample (250Hz sample rate)
  };

  // Average the calibration values for each axis
  gyro_x_cal /= CALIBRATION_SAMPLES;
  gyro_y_cal /= CALIBRATION_SAMPLES;
  gyro_z_cal /= CALIBRATION_SAMPLES;

  #ifdef DEBUG_IMU 
    String debugMsg = "[gyro_x_cal: " + String(gyro_x_cal)  
        + "] [gyro_y_cal: " + String(gyro_y_cal) + "] [gyro_z_cal: " + String(gyro_z_cal) + "]";  
    DEBUG_PRINT(DEBUG_IMU, debugMsg);
    DEBUG_PRINT(DEBUG_IMU, "IMU started.");  
  #endif
}


/**
 * @brief Calculates the pitch and roll angles from sensor data using gyroscope and accelerometer.
 * 
 * This function combines accelerometer and gyroscope data using complementary filters to 
 * calculate stable pitch and roll angles, reducing the effect of sensor noise and drift.
 */
void mpu6050Base::calculate() {

  read_mpu_6050_data();  // Read sensor data from the MPU6050

  // Subtract calibration offsets from raw gyro values
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  // Calculate pitch and roll angles using gyroscope data (integrating angular velocity)
  angle_pitch += gyro_x * 0.0000611;    // Increment pitch angle
  angle_roll += gyro_y * 0.0000611;     // Increment roll angle

  // Adjust pitch and roll if yaw rotation is detected (using gyroscope Z-axis)
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);  // Correct pitch with roll angle if yaw is present
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);   // Correct roll with pitch angle if yaw is present

  // Calculate the total accelerometer vector for normalization
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

  // Calculate pitch and roll angles from accelerometer data
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;  // Pitch angle from accelerometer
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;  // Roll angle from accelerometer

  // Apply accelerometer calibration offsets for pitch and roll
  angle_pitch_acc -= 0.0;   // Placeholder for pitch calibration value
  angle_roll_acc -= 0.0;    // Placeholder for roll calibration value

  // Combine accelerometer and gyroscope angles using a complementary filter
  if (set_gyro_angles) {
    angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;  // Fuse accelerometer and gyro pitch
    angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;     // Fuse accelerometer and gyro roll
  } else {
    // Initialize angles to accelerometer values if the IMU has just started
    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    set_gyro_angles = true;  // Set the flag indicating the IMU is initialized
  };

  // Apply complementary filter to dampen angle noise
  angle_pitch_output = angle_pitch_output * 0.9 + angle_pitch * 0.1;  // Smooth pitch angle output
  angle_roll_output = angle_roll_output * 0.9 + angle_roll * 0.1;     // Smooth roll angle output
}


/**
 * @brief Sets up the MPU6050 sensor registers for communication.
 * 
 * This function configures the MPU6050 for proper operation by initializing
 * the power management, accelerometer, and gyroscope registers.
 */
void mpu6050Base::setup_mpu_6050_registers() {  
  // Wake up MPU6050 from sleep mode

  Wire.beginTransmission(mpu6050_addr);
  Wire.write(0x6B);  // Power Management register (0x6B)
  Wire.write(0x01);  // Set bit 6 to 0 to wake up the MPU6050
  Wire.endTransmission();

  // Configure accelerometer for +/- 2g range
  Wire.beginTransmission(mpu6050_addr);
  Wire.write(0x1C);  // Accelerometer configuration register (0x1C)
  Wire.write(0x00);  // Set to +/- 2g full-scale range
  Wire.endTransmission();

  // Configure gyroscope for +/- 250 degrees/s range
  Wire.beginTransmission(mpu6050_addr);
  Wire.write(0x1B);  // Gyroscope configuration register (0x1B)
  Wire.write(0x00);  // Set to +/- 250 degrees/s full-scale range
  Wire.endTransmission();
  delay(20);  // Allow time for the configuration to take effect

}


/**
 * @brief Reads sensor data from the MPU6050 via I2C.
 * 
 * This function reads 14 bytes of sensor data (accelerometer, gyroscope, and temperature)
 * from the MPU6050 and stores it in the respective variables.
 */
void mpu6050Base::read_mpu_6050_data() {
  Wire.beginTransmission(mpu6050_addr);         // Start communication with MPU6050
  Wire.write(0x3B);                             // Starting register for accelerometer data
  Wire.endTransmission();
  Wire.requestFrom(mpu6050_addr, (uint8_t)14);  // Request 14 bytes of data
  
  if (Wire.available() >= 14) {
    // Read accelerometer and gyroscope data
    acc_x = Wire.read() << 8 | Wire.read();   // Combine high and low bytes for X axis accelerometer
    acc_y = Wire.read() << 8 | Wire.read();   // Combine high and low bytes for Y axis accelerometer
    acc_z = Wire.read() << 8 | Wire.read();   // Combine high and low bytes for Z axis accelerometer
    temperature = Wire.read() << 8 | Wire.read();  // Temperature data (2 bytes)
    gyro_x = Wire.read() << 8 | Wire.read();  // Combine high and low bytes for X axis gyroscope
    gyro_y = Wire.read() << 8 | Wire.read();  // Combine high and low bytes for Y axis gyroscope
    gyro_z = Wire.read() << 8 | Wire.read();  // Combine high and low bytes for Z axis gyroscope
  }
  else{
    ERROR_PRINT("Data cannot be read from MPU6050!");
  }

  #ifdef DEBUG_IMU 
    String debugMsg = "[acc_x: " + String(acc_x)  
        + "] [gyro_x: " + String(gyro_x-gyro_x_cal) + "] [gyro_z: " + String(gyro_z-gyro_z_cal) + "]";  
    DEBUG_PRINT(DEBUG_IMU, debugMsg);
  #endif
  
  return;
}

/**
 * @brief Gets the X-axis gyroscope value in degrees per second.
 * 
 * This function retrieves the X-axis gyroscope value, applies calibration offsets,
 * and scales the value to the correct unit of degrees per second. The scaling factor 
 * (131.0) is used to convert the raw sensor data into degrees per second, and 
 * a constant offset (128.1) is subtracted to adjust for any bias.
 * 
 * @return The X-axis gyroscope value in degrees per second.
 */
float mpu6050Base::getGyroX(){
  return (gyro_x - gyro_x_cal) / MPU6050_GYRO_SCALE;
}


/**
 * @brief Gets the Z-axis gyroscope value in degrees per second.
 * 
 * This function retrieves the Z-axis gyroscope value, applies calibration offsets,
 * and scales the value to degrees per second. The scaling factor (131.0) is used 
 * to convert the raw sensor data into degrees per second.
 * The result is negated since the sensor may output a negative value depending on
 * its orientation.
 * 
 * @return The Z-axis gyroscope value in degrees per second.
 */
float mpu6050Base::getGyroZ(){
  return - (gyro_z - gyro_z_cal) / MPU6050_GYRO_SCALE;
}

/**
 * @brief Calculates the pitch angle based on accelerometer data.
 * 
 * This function computes the pitch angle of the device by using the accelerometer
 * values on the Y and Z axes. The pitch angle is calculated using the `atan2` function
 * to account for both axes' contributions, and the result is converted from radians 
 * to degrees by multiplying by a factor of 57.3.
 * 
 * @return The pitch angle in degrees.
 */
float mpu6050Base::getPitchAngle(){
  return atan2(mpu.acc_y, mpu.acc_z) * 57.3;
}

/**
 * @brief Calculates the IMU sensor errors for calibration.
 * 
 * This function computes the error values for the accelerometer and gyroscope
 * by taking multiple readings while the sensor is stationary.
 * 
 * @return IMUErrorData Structure containing calculated accelerometer and gyroscope errors.
 */
IMUErrorData mpu6050Base::calculate_IMU_error() {
  IMUErrorData imuError;
  
  // Calibrate accelerometer by averaging multiple readings
  for (int i = 0; i < 100; i++) {
    read_mpu_6050_data();
    imuError.accel_x += acc_x;
    imuError.accel_y += acc_y;
    imuError.accel_z += acc_z;
    imuError.gyro_x += gyro_x;
    imuError.gyro_y += gyro_y;
    imuError.gyro_z += gyro_z;
  }

  // Average the readings to calculate errors
  imuError.accel_x /= 100.0;
  imuError.accel_y /= 100.0;
  imuError.accel_z /= 100.0;
  imuError.gyro_x /= 100.0;
  imuError.gyro_y /= 100.0;
  imuError.gyro_z /= 100.0;

  return imuError;  // Return the calculated error data
}
