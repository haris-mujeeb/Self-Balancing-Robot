#pragma once

// Global debug mode (set to false to disable all debugging)
#define DEBUG_MODE true      
// Module-specific debug modes
#define DEBUG_MOTOR true     // Debugging for Motor Control module
#define DEBUG_VOLTAGE true   // Debugging for Communication module
#define DEBUG_IMU true       // Debugging for IMU module
#define DEBUG_KALMAN true    // Debugging for Communication module
#define DEBUG_CONTROL true   // Debugging for Communication module
#define DEBUG_PID_PITCH true // Debugging for PID control for Pitch
#define DEBUG_PID_YAW true // Debugging for PID control for Pitch
#define DEBUG_PID_POSITION true // Debugging for PID control for Pitch

// Macro for conditional debugging
#if DEBUG_MODE
  #if TIMING
    #define DEBUG_PRINT(module, x) \
        if (module) { \
          Serial.println(String(millis())); \
        }
  #else
    #define DEBUG_PRINT(module, x) \
        if (module) { \
          Serial.print("[time: "); \
          Serial.print(String(millis())); \
          Serial.print("] "); \
          Serial.println(x); \
        }
  #endif
#else
  #define DEBUG_PRINT(module, x)
#endif

// Macro for error messages (always active)
#define ERROR_PRINT(x) Serial.println(x)