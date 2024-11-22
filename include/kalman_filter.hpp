#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Arduino.h>
#include "debugConfig.h"

/**
 * @class KalmanFilter
 * @brief A class that implements the Kalman filter for sensor fusion.
 * 
 * The Kalman filter is used to estimate the true state of the system (e.g., angle, angular velocity)
 * based on noisy sensor measurements. It fuses the accelerometer and gyroscope readings to provide
 * more accurate and stable estimates for applications like robot balance control.
 */
class KalmanFilter {
 private:
  float m_dt;           ///< Time step for the filter update (seconds)
  float m_Q_angle;      ///< Process noise covariance for angle
  float m_Q_gyro;       ///< Process noise covariance for gyro
  float m_R_angle;      ///< Measurement noise covariance for angle
  float m_C_0;          ///< Constant used in the complementary filter

  float q_bias;         ///< Estimated bias in the gyroscope
  float angle_err;      ///< Error between measured angle and estimated angle

  float Pdot[4] = {0, 0, 0, 0};   ///< Temporary matrix used in the time update
  float P[2][2] = {{1, 0}, {0, 1}}; ///< Covariance matrix of the state estimation
  float K_0 = 0;         ///< Kalman gain for angle
  float K_1 = 0;         ///< Kalman gain for gyroscope bias
  float angle_dot;       ///< Estimated angular velocity

 public:
  float angle; ///< Estimated angle of the system (e.g., robot tilt)

  /**
   * @brief Constructor to initialize the Kalman filter with specific parameters.
   * @param dt The time step (seconds) for the filter update.
   * @param Q_angle The process noise covariance for the angle.
   * @param Q_gyro The process noise covariance for the gyroscope.
   * @param R_angle The measurement noise covariance for the angle.
   * @param C_0 The constant used in the complementary filter.
   */
  KalmanFilter(float dt, float Q_angle, float Q_gyro, float R_angle, float C_0) {
    m_dt = dt;
    m_Q_angle = Q_angle;
    m_Q_gyro = Q_gyro;
    m_R_angle = R_angle;
    m_C_0 = C_0;
  } 

  /**
   * @brief Updates the Kalman filter with new sensor measurements and returns the estimated angle.
   * 
   * This method performs both the time update (prediction) and the measurement update (correction)
   * based on the sensor readings (measured angle and measured gyro).
   * 
   * @param measured_angle The measured angle from the accelerometer.
   * @param measured_gyro The measured angular velocity from the gyroscope.
   * @return The filtered angle estimate.
   */
  float getAngle(float measured_angle, float measured_gyro) {  
    // Prediction step: Update the estimated angle based on the gyro reading and bias correction
    angle += (measured_gyro - q_bias) * m_dt;
    angle_err = measured_angle - angle; // Error calculation between the measured and predicted angle

    // Time update (prediction) of the covariance matrix
    Pdot[0] = m_Q_angle - P[0][1] - P[1][0];
    Pdot[1] = -P[1][1];
    Pdot[2] = -P[1][1];
    Pdot[3] = m_Q_gyro;
    P[0][0] += Pdot[0] * m_dt;
    P[0][1] += Pdot[1] * m_dt;
    P[1][0] += Pdot[2] * m_dt;
    P[1][1] += Pdot[3] * m_dt;

    // Calculate the Kalman gain
    double C0_P00 = m_C_0 * P[0][0];
    double C0_P01 = m_C_0 * P[0][1];
    double E = m_R_angle + m_C_0 * C0_P00;

    // Update the Kalman gains
    K_0 = m_C_0 * P[0][0] / E;
    K_1 = m_C_0 * P[1][0] / E;

    // Measurement update (correction) of the covariance matrix
    P[0][0] -= K_0 * C0_P00;
    P[0][1] -= K_0 * C0_P01;
    P[1][0] -= K_1 * C0_P00;
    P[1][1] -= K_1 * C0_P01;

    // Correct the angle estimate using the Kalman gain and the angle error
    angle += K_0 * angle_err;
    q_bias += K_1 * angle_err;
    angle_dot = measured_angle - q_bias;

    // Debugging output for Kalman filter state
  #ifdef DEBUG_KALMAN
    String debugMsg = "[Debug] [Kalman values] [angle:" + String(angle) +
                "] [q_bias:" + String(q_bias) + "] [angle_dot: " + String(angle_dot) + "]";
    DEBUG_PRINT(DEBUG_KALMAN, debugMsg);
  #endif

    return angle;  // Return the filtered angle estimate
  }
};

// Declare the global KalmanFilter instance to be used across the program
extern KalmanFilter kalman;

#endif // KALMANFILTER_H
