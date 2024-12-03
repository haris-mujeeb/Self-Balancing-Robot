#pragma once

// plot mode (for plotting pitch angle , yaw angle and position along with there correspoing pid output values)
#define PLOT_MODE false      

// Global debug mode (set to false to disable all debugging)
#define DEBUG_MODE true     
#if DEBUG_MODE
  // Module-specific debug modes
  #define DEBUG_MOTOR false     // Debugging for Motor Control module
  #define DEBUG_IMU false       // Debugging for IMU module
  #define DEBUG_KALMAN false    // Debugging for Communication module
  #define DEBUG_CONTROL false   // Debugging for Communication module
  #if DEBUG_CONTROL
    #define DEBUG_VOLTAGE false   // Debugging for Communication module
    #define DEBUG_PID_PITCH false // Debugging for PID control for Pitch
    #define DEBUG_PID_YAW false // Debugging for PID control for Pitch
    #define DEBUG_PID_POSITION false  // Debugging for PID control for Pitch
    #define DEBUG_ENCODER false  // Debugging for Motor incremental encoders
    #define DEBUG_WATCHDOG false
  #endif
  #define DEBUG_COMM true
#endif

// Macro for conditional debugging
#if DEBUG_MODE
#define DEBUG_PRINT(module, x) \
    if (module) { \
      Serial.print("[time:"); \
      Serial.print(String(millis())); \
      Serial.print("ms][Debug] "); \
      Serial.println(x); \
    }
#else
  #define DEBUG_PRINT(module, x)
#endif

#if PLOT_MODE
  #define SEND_FOR_PLOT(x) Serial.println(x);
#else
  #define SEND_FOR_PLOT(x)
#endif

// Macro for error messages (always active)
#define ERROR_PRINT(x) \
  do { \
    Serial.print("[time:"); \
    Serial.print(millis()); \
    Serial.print("ms][ERROR]"); \
    Serial.println(x); \
  } while(0)