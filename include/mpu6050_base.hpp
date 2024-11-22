#pragma once

#include "Arduino.h"
#include "HardwareSerial.h"
#include "Wire.h"
#include "debugConfig.h"

// Macro for debug messages
#if DEBUG_IMU
  #define DEBUG_MSG(x) Serial.println(x)
#else
  #define DEBUG_MSG(x)
#endif

// Define constants for MPU6050 configuration
#define MPU6050_READINGS 200        ///< Number of readings for error calculation
#define MPU6050_ACCEL_SCALE 16384.0 ///< Accelerometer scale (±2g)
#define MPU6050_GYRO_SCALE 131.0    ///< Gyroscope scale (±250deg/s)
#define CALIBRATION_SAMPLES 10    ///< Number of samples used for calibration

/**
 * @struct IMUErrorData
 * @brief Stores accelerometer and gyroscope error data.
 * 
 * This structure holds the error values calculated for the
 * accelerometer and gyroscope for each axis (X, Y, Z).
 */
struct IMUErrorData {
    float accel_x = 0;  ///< Accelerometer error in the X axis
    float accel_y = 0;  ///< Accelerometer error in the Y axis
    float accel_z = 0;  ///< Accelerometer error in the Z axis
    float gyro_x = 0; ///< Gyroscope error in the X axis
    float gyro_y = 0; ///< Gyroscope error in the Y axis
    float gyro_z = 0; ///< Gyroscope error in the Z axis
};


/**
 * @class mpu6050_base
 * @brief Base class for handling MPU6050 sensor data.
 * 
 * This class provides methods for initializing the MPU6050 sensor, 
 * reading accelerometer and gyroscope data, calculating angles,
 * and performing sensor calibration and error correction.
 */
class mpu6050_base {
 private:
  uint8_t mpu6050_addr;  ///< I2C address of MPU6050
  long loop_timer;       ///< Timer for maintaining control loop frequency

  // Helper functions for internal use
  void setup_mpu_6050_registers(); ///< Initializes MPU6050 registers
  void printErrorData(const IMUErrorData &imuError, HardwareSerial &serialPort); ///< Prints error data to serial port

public:
  /**
   * @brief Constructor for mpu6050_base class.
   * 
   * @param i2c_addr I2C address of the MPU6050 sensor (default 0x68).
   */
  mpu6050_base(uint8_t i2c_addr = 0x68) : mpu6050_addr(i2c_addr){}

  float acc_x = 0;                     ///< Accelerometer data in the X axis
  float acc_y = 0;                     ///< Accelerometer data in the Y axis
  float acc_z = 0;                     ///< Accelerometer data in the Z axis
  float temperature = 0;                      ///< Temperature data from the MPU6050
  float gyro_x = 0;                    ///< Gyroscope data in the X axis
  float gyro_y = 0;                    ///< Gyroscope data in the Y axis
  float gyro_z = 0;                    ///< Gyroscope data in the Z axis

  /**
   * @brief Initializes the MPU6050 sensor and calibrates the gyroscope.
   * 
   * This function sets up the MPU6050 registers and performs gyroscope 
   * calibration by averaging a set of samples to find the gyro offset.
   */
  void init();

  /**
   * @brief Reads raw data from the MPU6050 sensor.
   * 
   * This function communicates with the MPU6050 over I2C to read 
   * accelerometer and gyroscope data from the sensor's registers.
   */
  void read_mpu_6050_data();

  /**
   * @brief Calculates pitch and roll angles from sensor data.
   * 
   * This function processes the accelerometer and gyroscope data to calculate 
   * the current pitch and roll angles. It also applies a complementary filter 
   * to reduce the effect of gyro drift and noise.
   */
  void calculate();

  /**
   * @brief Calculates IMU sensor error for accelerometer and gyroscope.
   * 
   * This function calculates the error values by averaging multiple readings
   * from the accelerometer and gyroscope when the IMU is stationary. The error 
   * values are used to correct subsequent sensor readings.
   * 
   * @return IMUErrorData Struct containing accelerometer and gyroscope errors.
   */
  IMUErrorData calculate_IMU_error();

  /**
   * @brief Calculates IMU sensor error and prints results to serial.
   * 
   * This function calculates the IMU error and prints the result to the 
   * specified serial port for debugging or calibration purposes.
   * 
   * @param serialPort Reference to the serial port for output.
   * @return IMUErrorData Struct containing accelerometer and gyroscope errors.
   */
  IMUErrorData calculate_IMU_error(HardwareSerial &serialPort);

  /**
   * @brief Recalibrates the MPU6050 for the pitch axis.
   * 
   * This function recalibrates the MPU6050 sensor by averaging multiple 
   * readings for the pitch angle and angular velocity when the sensor is 
   * stationary, setting new zero values.
   */
  void recalibrate();

  /**
   * @brief Recalibrates the MPU6050 and prints results to serial.
   * 
   * This function performs recalibration and prints the resulting zero 
   * angle and angular velocity values to the specified serial port.
   * 
   * @param serialPort Reference to the serial port for output.
   */
  void recalibrate(HardwareSerial &serialPort);
};

extern mpu6050_base mpu;