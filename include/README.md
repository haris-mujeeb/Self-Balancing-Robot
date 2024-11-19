# Pinout
<img src= "https://github.com/haider-rizvi-github/Self-Balancing-Robot/blob/main/reference%20material/Pin%20ports.png"  width="520">

# Motor Drive
To save IO pins we use TB6612FNG (H-bridge) in conjunction with SN74LVC2G14 (Two-Way Schmitt Trigger Reverser) like below 
<img src= "https://github.com/user-attachments/assets/eca88126-7f42-417b-86ec-745b861a1531"  width="480">
<img src= "https://github.com/user-attachments/assets/c2305d4b-a919-4fbb-9a67-0044d94804b9"  width="240">

# Inertial Measuring Unit:
In this project [MPU6050](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf) is used as IMU.

## Power Managment 1 Config
![image](https://github.com/user-attachments/assets/1631a5d6-966e-4ab5-8da3-172b7c94a8c8)

(ref: from [datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf))

- Using PLL with x-axis gyroscope reference (e.g. CLKSEL = 0x01)
- Sleep Mode Disabled (SLEEP = 0x00)

```C
  // Start communicating with MPU6050
  Wire.beginTransmission(mpu6050_addr);
  Wire.write(0x6B);  // Talk to the Power Management register 0x6B
  Wire.write(0x01);  // setup to wake up the register
  Wire.endTransmission(); //end the transmission
```

## Gyro and Accel Config
![image](https://github.com/user-attachments/assets/cdf3c9ea-3c96-42db-9e55-ea67516756b6)

![image](https://github.com/user-attachments/assets/b6746afb-7522-4cbb-b851-032bca284a99)

![image](https://github.com/user-attachments/assets/cf81da80-db9f-4f58-978d-0b9b63bcb6df)

![image](https://github.com/user-attachments/assets/ea1f5220-99a0-4bd3-b4ae-92d5fab33c65)

(ref: from [datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf))

- Full scale range as $\pm$ 250 deg/s for gyro and $\pm$ 2g for accel  (e.g. GYRO_CONFIG = 0x00 and ACCEL_CONFIG = 0x00)

```C
  // Configuring the accelerometer (+/- 8g)
  Wire.beginTransmission(mpu6050_addr);
  Wire.write(0x1C);  // talk to ACCEL_CONFIG register: 1C
  Wire.write(0x00);  //  (+/- 2g full scale range)
  Wire.endTransmission();

  // Configuring gyroscope (500dps full scale)
  Wire.beginTransmission(mpu6050_addr);
  Wire.write(0x1B);  // talk to GYRO_CONFIG register: 1B
  Wire.write(0x00);  //  (250deg/s full scale)
  Wire.endTransmission();
```
 
## Reading Values
![image](https://github.com/user-attachments/assets/fdfc4e3b-b1a0-46d9-923b-9a12f1bc4b09)

![image](https://github.com/user-attachments/assets/d629bfd3-3008-4c23-b294-aba11e6e66e8)

![image](https://github.com/user-attachments/assets/d2778dea-06a7-4f35-9722-856238d07899)

(ref: from [datasheet](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf))

```C
  // Read the raw gyro and accelerometer data
  // Start communicating with the MPU-6050
  Wire.beginTransmission(mpu6050_addr);
  Wire.write(0x3B);
  Wire.endTransmission();
  // Request 14 bytes from the MPU 6050  to get data from ACCEL_XOUT_H -> GYRO_ZOUT_L (check register sheet of MPU)
  Wire.requestFrom(mpu6050_addr, (uint8_t)14);

  // Wait until all the bytes are recieved
  if (Wire.available() >= 14) {
    // using condition to store the data
    acc_x = Wire.read() << 8 | Wire.read();  // first condition shifts the high bit to 8 digits and second condition places the LOW bit to complete data
    acc_y = Wire.read() << 8 | Wire.read();
    acc_z = Wire.read() << 8 | Wire.read();
    temp = Wire.read() << 8 | Wire.read();
    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();
  };
```

## Gyro drift
we can calibrate gyro in order to offset gyro drift.

```C
  // Read the raw acc and gyro data from the MPU_6050 1000 times
  for (int cal_int = 0; cal_int < 1000; cal_int++) {
    read_mpu_6050_data();
    // calibrating the gyro
    gyro_x_cal += gyro_x;   //add the gyro x offset to the gyro_x_cal variable
    gyro_y_cal += gyro_y;   //add the gyro y offset to the gyro_y_cal variable
    gyro_z_cal += gyro_z;   //add the gyro z offset to the gyro_z_cal variable
    delay(3);  //setting delay 3us to have 250Hz for-loop
  };

  // Getting average offset by dividing results with 1000
  gyro_x_cal = gyro_x_cal / 1000;
  gyro_y_cal = gyro_y_cal / 1000;
  gyro_z_cal = gyro_z_cal / 1000;
```
## Enable Interrupt
We can measure DC motor relative rotation using  embedded hall sensor feedback signals. This can be done enabling interrupt pin on Pin 2 & 4. Using using [EnableInterrupt.h](https://github.com/GreyGnome/EnableInterrupt) library (Author: Mike "GreyGnome" Schwager) one can enable interrupt on any arduino pin. 
```C
  enableInterrupt(ENCODER_LEFT_A_PIN|PINCHANGEINTERRUPT, encoderCounterLeftA, CHANGE);
  enableInterrupt(ENCODER_RIGHT_A_PIN, encoderCounterRightA, CHANGE);
```
