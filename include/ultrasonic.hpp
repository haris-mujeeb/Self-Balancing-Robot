#pragma once
#include <stdint.h>
#include "Arduino.h"
#include "pins.hpp"
#include "debugConfig.h"
#include "EnableInterrupt.h"


/**
 * @brief Enum for ultrasonic sensor measurement states.
 */
enum UltrasonicMeasureFlagStatus {
    SEND,      /**< Measurement signal sent */
    RECEIVED,  /**< Measurement signal received */
    IDLE    /**< Idle state */
};

// Debugging
#if DEBUG_USONIC
  String debugMsg = "";
#endif

// Constants
constexpr uint8_t IR_COUNT_DELAY_MS = 50; 
constexpr uint8_t IR_PULSE_DELAY_MS = 15; 
constexpr uint8_t SLIDING_WINDOW_SIZE = 10;
constexpr uint8_t USONIC_GET_DISTANCE_DELAY_MS = 100; 
constexpr float SPEED_OF_SOUND_HALVED = (340.29 * 100.0)/(2 * 1000.0 * 1000.0); // in cm per microSec

// Global Variables
unsigned long irSendTime = 0;
unsigned long irLeftCountTime = 0;
unsigned long irRightCountTime = 0;
unsigned long usonicMeasurePrevTime = 0; 
unsigned long usonicGetDistancePrevTime = 0; 

bool irLeftReceiveFlag = false;
bool irRightReceiveFlag = false;
uint8_t irRightPulseCount = 0;
uint8_t irRightRunningCount = 0;
uint8_t irLeftPulseCount = 0;
uint8_t irLeftRunningCount = 0;
bool irLeftIsObstacle = false;
bool irRightIsObstacle = false;

bool irLeftHistory[SLIDING_WINDOW_SIZE] = {false};
bool irRightHistory[SLIDING_WINDOW_SIZE] = {false};
uint8_t irLeftIndex = 0;
uint8_t irRightIndex = 0;

UltrasonicMeasureFlagStatus usonicMeasureFlag = IDLE; 
uint8_t usonicDistanceValue = 0; 
 
// Function Prototypes
void IRLeftReceive();
void IRRightReceive();
void IRSesorSend38KPule(unsigned char ir_pin);
void CheckIRObstacle();
void IRCheckObstacle(unsigned long currentTime, int &count, int);
void ProcessLeftIRSensor();
void ProcessRightIRSensor();
void UpdateSlidingWindow(bool value, bool* history, uint8_t& index, uint8_t& runningCount);
void StartUltrasonicMeasurement();
void HandleUltrasonicMeasurementInterrupt();

/**
 * @brief Initializes pins and components for IR and Ultrasonic sensors.
 */
void InitializeIRAndUltrasonic(){
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(IR_SEND_PIN, OUTPUT);
  pinMode(LEFT_RECEIVE_PIN, INPUT_PULLUP);
  pinMode(RIGHT_RECEIVE_PIN, INPUT_PULLUP);
  enableInterrupt(LEFT_RECEIVE_PIN, IRLeftReceive, FALLING);
  enableInterrupt(RIGHT_RECEIVE_PIN, IRRightReceive, FALLING);
  DEBUG_PRINT(DEBUG_USONIC, "IR and Ultrasonic Sensors Initialized");
}


/**
*@brief Receive pin interrupt service function
*
*/
void IRLeftReceive(){
  if(irLeftReceiveFlag == false) {
    irLeftReceiveFlag  = true;
    enableInterrupt(LEFT_RECEIVE_PIN, IRLeftReceive, RISING);
  } else if (irLeftReceiveFlag == true) {
    irLeftPulseCount++;
    irLeftReceiveFlag = false;
    enableInterrupt(LEFT_RECEIVE_PIN, IRLeftReceive, FALLING);
  }
} 

/**
 * @brief Handles the interrupt for the right IR sensor.
 */
void IRRightReceive(){
  if(irRightReceiveFlag == false) {
    irRightReceiveFlag  = true;
    enableInterrupt(RIGHT_RECEIVE_PIN, IRRightReceive, RISING);
  } else if (irRightReceiveFlag == true) {
    irRightPulseCount++;
    irRightReceiveFlag = false;
    enableInterrupt(RIGHT_RECEIVE_PIN, IRRightReceive, FALLING);
  }  
}


/**
 * @brief Sends a 38kHz pulse on the given pin.
 * @param pin Pin number to send the IR pulse.
 */
void IRSesorSend38KPule(unsigned char ir_pin){
  for( int i = 0; i < 39; i++) {
    digitalWrite(ir_pin, LOW);
    delayMicroseconds(9);
    digitalWrite(ir_pin, HIGH);
    delayMicroseconds(9);
  }
}

/**
 * @brief Checks for obstacles using IR sensors.
 */
void CheckIRObstacle(){
  if (millis() - irSendTime > IR_PULSE_DELAY_MS) {
    IRSesorSend38KPule(IR_SEND_PIN);
    irSendTime = millis();
  }
  ProcessLeftIRSensor();
  ProcessRightIRSensor();
  DEBUG_PRINT(DEBUG_USONIC && irLeftIsObstacle, "Obstacle on Left");
  DEBUG_PRINT(DEBUG_USONIC && irRightIsObstacle, "Obstacle on Right");
}

/**
 * @brief Processes the left IR sensor for obstacle detection.
 */
void ProcessLeftIRSensor(){
  if (millis() - irLeftCountTime > IR_COUNT_DELAY_MS) {
    UpdateSlidingWindow(irLeftPulseCount >= 3, irLeftHistory, irLeftIndex, irLeftRunningCount);    
    irLeftIsObstacle = (irLeftRunningCount >= 5);
    irLeftPulseCount = 0;
    irLeftCountTime = millis();
  }
}

/**
 * @brief Processes the right IR sensor for obstacle detection.
 */
void ProcessRightIRSensor(){
  if (millis() - irRightCountTime > IR_COUNT_DELAY_MS) {
    UpdateSlidingWindow((irRightPulseCount >= 3), irRightHistory, irRightIndex, irRightRunningCount);
    irRightIsObstacle = (irRightRunningCount >= 5);
    irRightPulseCount = 0;
    irRightCountTime = millis();
  }
}


/**
 * @brief Updates the sliding window for filtering sensor data.
 * @param value New sensor value.
 * @param history Sliding window history array.
 * @param index Current index in the sliding window.
 * @param runningCount Running count of active values in the window.
 */
void UpdateSlidingWindow(bool value, bool* history, uint8_t& index, uint8_t& runningCount) {
  runningCount -= history[index];
  runningCount += value;
  history[index] = value;
  index  = (index + 1) % SLIDING_WINDOW_SIZE; 
}

/**
 * @brief Starts an ultrasonic distance measurement.
 */
void StartUltrasonicMeasurement(){
  if(millis() - usonicGetDistancePrevTime > USONIC_GET_DISTANCE_DELAY_MS){
    usonicMeasureFlag = SEND;
    usonicGetDistancePrevTime = millis();

    attachPinChangeInterrupt(ECHO_PIN, HandleUltrasonicMeasurementInterrupt, RISING);

    digitalWrite(TRIG_PIN, LOW);
    // delay(2);
    digitalWrite(TRIG_PIN, HIGH);
    // delay(10);
    digitalWrite(TRIG_PIN, LOW);
    #if DEBUG_USONIC 
      debugMsg += "usonicDist:";
      debugMsg += usonicDistanceValue;
      DEBUG_PRINT(DEBUG_USONIC, debugMsg);
      debugMsg = "";
    #endif
  }
}

/**
 * @brief Handles the ultrasonic sensor measurement interrupt.
 */
void HandleUltrasonicMeasurementInterrupt(){
  if(usonicMeasureFlag == SEND){
    usonicMeasurePrevTime = micros();
    attachPinChangeInterrupt(ECHO_PIN, HandleUltrasonicMeasurementInterrupt, FALLING);
    usonicMeasureFlag = RECEIVED;
  } else if(usonicMeasureFlag == RECEIVED){
    usonicDistanceValue = (uint8_t)((micros() - usonicMeasurePrevTime) * SPEED_OF_SOUND_HALVED); 
    usonicMeasureFlag = IDLE; 
  }
}
