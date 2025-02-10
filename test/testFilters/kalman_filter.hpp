
class KalmanFilter {
 public:
  KalmanFilter(float dt, float Q_angle, float Q_gyro, float R_angle, float C_0);
  double getAngle(double measured_angle, double measured_gyro);
  double angle;

 private:
  float m_dt, m_Q_angle, m_Q_gyro, m_R_angle, m_C_0;
  float q_bias, angle_err;
  float Pdot[4] = {0,0,0,0};
  float P[2][2] = {{1, 0}, {0, 1}};
  float K_0 = 0;
  float K_1 = 0;
  float angle_dot;

};

