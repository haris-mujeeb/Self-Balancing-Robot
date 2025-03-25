#pragma once

// Global debug mode (set to false to disable all debugging)
#define PLOT_MODE true      
#define DEBUG_MODE true     
// Module-specific debug modes
#if DEBUG_MODE
  #define DEBUG_MOTOR false     // Debugging for Motor Control module
  #define DEBUG_VOLTAGE false   // Debugging for Communication module
  #define DEBUG_IMU false       // Debugging for IMU module 
  #define DEBUG_KALMAN false    // Debugging for Communication module
  #define DEBUG_CONTROL false   // Debugging for Communication module
  #define DEBUG_PID_PITCH false // Debugging for PID control for Pitch
  #define DEBUG_PID_YAW false // Debugging for PID control for Pitch
  #define DEBUG_PID_POSITION false  // Debugging for PID control for Pitch
  #define DEBUG_ENCODER false  // Debugging for Motor incremental encoders
#endif
// Macro for conditional debugging
#if DEBUG_MODE
  #define DEBUG_PRINT(module, x) \
      if (module) { \
        Serial.print("[time: "); \
        Serial.print(String(millis())); \
        Serial.print("] "); \
        Serial.println(x); \
      }
#else
  #define DEBUG_PRINT(module, x)
#endif

#if PLOT_MODE
  #define SEND_FOR_PLOT(module, x) \
      if (module) { Serial.println(x); }
#else
  #define SEND_FOR_PLOT(module, x)
#endif

// Macro for error messages (always active)
#define ERROR_PRINT(x) Serial.println(x)