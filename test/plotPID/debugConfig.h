#pragma once

// Global debug mode (set to false to disable all debugging)
#define DEBUG_MODE false      
#define TIMING false
// Module-specific debug modes
#define DEBUG_CONTROL false    // Debugging for Communication module
#define DEBUG_MOTOR false   // Debugging for Motor Control module
#define DEBUG_VOLTAGE false  // Debugging for Communication module
#define DEBUG_IMU false     // Debugging for IMU module
#define DEBUG_KALMAN false  // Debugging for Communication module

// Macro for conditional debugging
#if DEBUG_MODE
  #if TIMING
    #define DEBUG_PRINT(module, x) \
        if (module) { \
          Serial.println(x); \
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