#include "mpu6050_base.hpp"

void mpu6050_base::init() {
  setup_mpu_6050_registers();

    // Gyro calibration+++++++++++++++++++++++++++++
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;   //add the gyro x offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;   //add the gyro y offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;   //add the gyro z offset to the gyro_z_cal variable
    delay(3);  //250Hz sample rate
  };

  // Average calibration values
  gyro_x_cal = gyro_x_cal / CALIBRATION_SAMPLES;
  gyro_y_cal = gyro_y_cal / CALIBRATION_SAMPLES;
  gyro_z_cal = gyro_z_cal / CALIBRATION_SAMPLES;

  // Initialize timer for control loop
  loop_timer = micros();  // To calculate microseconds
  DEBUG_PRINT(DEBUG_IMU, "[Debug] IMU started.");  
}


void mpu6050_base::calculate(){
  read_mpu_6050_data();

  //Subtract the offset values from the raw gyro values
  gyro_x -= gyro_x_cal;   // gyro_x_cal is the offset got by averaging 1000 values
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  // Gyro angle calculations 
  // NOTE: 0.0000611 = 1 / (250 Hz x 65.5)
  angle_pitch += gyro_x * 0.0000611;    // Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += gyro_y * 0.0000611;     // Calculate the traveled roll and add this to the angle_roll variable

  //NOTE: 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  angle_pitch += angle_roll * sin(gyro_z * 0.000001066);    //If the IMU has yawed transfer the roll angle to the pitch angle
  angle_roll -= angle_pitch * sin(gyro_z * 0.000001066);    //If the IMU has yawed transfer the pitch angle to the roll angle

  // Accelerometer angle calculations
  // NOTE: 57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z)); // Calculate the total accelerometer vector
  angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;   // Calculate the pitch angle
  angle_roll_acc = asin((float)acc_x / acc_total_vector) * -57.296;   // Calculate the roll angle

  angle_pitch_acc -= 0.0;   // Accelerometer calibration value for pitch
  angle_roll_acc -= 0.0;    // Accelerometer calibration value for roll

  if (set_gyro_angles) {  // if the IMU has been running
    angle_pitch = angle_pitch*0.9996 + angle_pitch_acc*0.0004;  // Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    angle_roll = angle_roll*0.9996 + angle_roll_acc*0.0004;     // Correct the drift of the gyro roll angle with the accelerometer roll angle
  } else { // IMU has just started
    angle_pitch = angle_pitch_acc;  // Set the gyro pitch angle equal to the accelerometer pitch angle
    angle_roll = angle_roll_acc;    // Set the gyro roll angle equal to the accelerometer roll angle
    set_gyro_angles = true;         // Set the IMU started flag 
  };

  // To dampen the pitch and roll angles a complementary filter is used
  angle_pitch_output = angle_pitch_output*0.9 + angle_pitch*0.1;  // Take 90% of the output pitch value and add 10% of the raw pitch 
  angle_roll_output = angle_roll_output*0.9 + angle_roll*0.1;     // Take 90% of the output roll value and add 10% of the raw roll
  
  while(micros() - loop_timer < 4000); // Wait for 250Hz
  
  loop_timer = micros(); // Reset the loop timer
}


void mpu6050_base::setup_mpu_6050_registers() {
  // Start communicating with MPU6050
  Wire.beginTransmission(mpu6050_addr);
  Wire.write(0x6B);  // Talk to the Power Management register 0x6B
  Wire.write(0x01);  // setup to wake up the register
  Wire.endTransmission(); //end the transmission

  // Configuring the accelerometer
  Wire.beginTransmission(mpu6050_addr);
  Wire.write(0x1C);  // talk to ACCEL_CONFIG register: 1C
  Wire.write(0x00);  //  (+/- 2g full scale range)
  Wire.endTransmission();

  // Configuring gyroscope
  Wire.beginTransmission(mpu6050_addr);
  Wire.write(0x1B);  // talk to GYRO_CONFIG register: 1B
  Wire.write(0x00);  // (+/- 250deg/s full scale)
  Wire.endTransmission();
  delay(20);
};


void mpu6050_base::read_mpu_6050_data() {
  Wire.beginTransmission(mpu6050_addr);         // Start communicating with the MPU-6050
  Wire.write(0x3B);                             // Starting register
  Wire.endTransmission();
  Wire.requestFrom(mpu6050_addr, (uint8_t)14);  // Request 14 bytes

  if (Wire.available() >= 14) {               // Wait until all the bytes are recieved
    // using condition to store the data
    acc_x = Wire.read() << 8 | Wire.read();   // shifts left the high bit by 8 digits and OR it with the LOW bit to complete data
    acc_y = Wire.read() << 8 | Wire.read();
    acc_z = Wire.read() << 8 | Wire.read();
    temp = Wire.read() << 8 | Wire.read();
    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();
    DEBUG_PRINT(DEBUG_IMU, "[Debug] data read from MPU6060 sucessfull.");
  };
};


IMUErrorData mpu6050_base::calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times

  IMUErrorData imuError;
  for (int i = 0; i < MPU6050_READINGS; i++) {
    read_mpu_6050_data();
    imuError.AccErrorX += (atan((acc_y) / sqrt(pow((acc_x), 2) + pow((acc_z), 2))) * 180 / PI);
    imuError.AccErrorY += (atan(-1 * (acc_x) / sqrt(pow((acc_y), 2) + pow((acc_z), 2))) * 180 / PI);
    imuError.GyroErrorX += gyro_x / MPU6050_GYRO_SCALE;
    imuError.GyroErrorY += gyro_y / MPU6050_GYRO_SCALE;
    imuError.GyroErrorZ += gyro_z / MPU6050_GYRO_SCALE;
  }

  imuError.AccErrorX /= MPU6050_READINGS;
  imuError.AccErrorY /= MPU6050_READINGS;
  imuError.GyroErrorX /= MPU6050_READINGS;
  imuError.GyroErrorY /= MPU6050_READINGS;
  imuError.GyroErrorZ /= MPU6050_READINGS;

  String debugMsg = "[Debug] imu error data " 
    + String(imuError.AccErrorX)  + ", " + String(imuError.AccErrorY)
    + ", " + String(imuError.AccErrorZ) + ", " + String(imuError.GyroErrorX)
    + ", " + String(imuError.GyroErrorY) + ", " + String(imuError.GyroErrorZ);
  DEBUG_PRINT(DEBUG_IMU, debugMsg);
  return imuError;
}


IMUErrorData mpu6050_base::calculate_IMU_error(HardwareSerial &serialPort) {
  IMUErrorData imuError = mpu6050_base::calculate_IMU_error(); 
  // Print the error values on the Serial Monitor
  String debugMsg = "[Debug] IMU Error Data: AccErrorX=" + String(imuError.AccErrorX)
                + ", AccErrorY=" + String(imuError.AccErrorY)
                + ", AccErrorZ=" + String(imuError.AccErrorZ)
                + ", GyroErrorX=" + String(imuError.GyroErrorX)
                + ", GyroErrorY=" + String(imuError.GyroErrorY)
                + ", GyroErrorZ=" + String(imuError.GyroErrorZ);

  DEBUG_PRINT(DEBUG_IMU ,debugMsg);
  return imuError;
}


void mpu6050_base::recalibrate() {
  long sum_angle = 0;
  long sum_angular_velocity = 0;
  const int calibration_samples = 1000;

  for (int i = 0; i < calibration_samples; i++) {
    read_mpu_6050_data();
    sum_angle += angle_pitch;             // Sum of pitch angles
    sum_angular_velocity += gyro_x;       // Sum of angular velocities
    delay(3);
  }
  // Calculate the average zero angle and angular velocity
  angle_zero = sum_angle / calibration_samples;
  angular_velocity_zero = sum_angular_velocity / calibration_samples;
}


void mpu6050_base::recalibrate(HardwareSerial &serialPort) {
  recalibrate();
  String debugMsg = String("[Debug] Re-calibration complete ") + 
  String("[Zero Angle: ") + String(angle_zero) + String("] [Zero Angular Velocity: ") 
  + String(angular_velocity_zero) + String("]");
  DEBUG_PRINT(DEBUG_IMU, debugMsg);
}