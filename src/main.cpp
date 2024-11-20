#include "debugConfig.h"
#include "motion_controller.hpp"

// //Setting PID parameters
// double kp_balance = 55, kd_balance = 0.75;
// double kp_speed = 10, ki_speed = 0.26;
// double kp_turn = 2.5, kd_turn = 0.5;
// double data = 0;

// TB6612FNG motor = TB6612FNG(STBY_PIN, AIN1, BIN1, PWMA_LEFT, PWMB_RIGHT);
// // PIDController pid = PIDController(kp_balance, 0, kd_balance, -3000.0f, 3000.0f);
// PIDController pid = PIDController(kp_speed, ki_speed, 0, -3000.0f, 3000.0f);
// mpu6050_base mpu;

motion_controller motion;

void setup() {
  // put your setup code here, to run once:
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);
  Serial.begin(250000);
  Wire.begin();
  // mpu.init();
  motion.init();
  // DEBUG_PRINT(DEBUG_MODE, "[Debug] Press the buttton to start!");
  // while(digitalRead(KEY_MODE)); // Stop execuation until Push button is pressed.
  // DEBUG_PRINT(DEBUG_MODE, "[Debug] Starting the loop...");
}

void loop() {

}