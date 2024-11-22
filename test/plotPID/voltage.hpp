#include "Arduino.h"          // Include Arduino core library for basic functions
#include <stdint.h>           // Include standard integer types for fixed-size data types
#include "pins.hpp"           // Include custom header for pin definitions
#include "debugConfig.h"      // Include debug configuration for serial output

// External variable declaration
extern float last_value;      ///< Last read voltage value, used for tracking the previous state

/**
 * @brief Initializes the voltage sensing module.
 * 
 * This function sets up the necessary hardware configurations for reading voltage values,
 * such as initializing the analog pin for voltage sensing, configuring resolution, or 
 * setting other necessary parameters for accurate voltage measurements.
 */
void voltage_init() {
    // Initialization logic for the voltage reading hardware.
    // For example, setting up the pinMode for voltage input, configuring ADC settings, etc.
    pinMode(VOLTAGE_PIN, INPUT);   // Set the voltage sensing pin as input
    analogReadResolution(12);      // Set the analog resolution (e.g., 12-bit ADC)
    last_value = 0;                // Initialize the last_value to 0 (or an appropriate baseline)
    
    #ifdef DEBUG_VOLTAGE  // If voltage debug mode is enabled in the configuration
        DEBUG_PRINT(DEBUG_VOLTAGE, "Voltage sensor initialized.");
    #endif
}

/**
 * @brief Reads the voltage value from the sensor.
 * 
 * This function reads the current voltage value from the specified sensor pin. It uses the 
 * ADC (Analog-to-Digital Converter) to convert the analog signal into a digital value, 
 * then calculates and returns the corresponding voltage based on the system's reference voltage.
 * 
 * @return float The measured voltage value in volts.
 */
float voltage_read() {
    // Read the raw analog value from the voltage sensor pin
    int sensor_value = analogRead(VOLTAGE_PIN);

    // Convert the sensor value to actual voltage based on reference voltage (assumes 3.3V system)
    float voltage = sensor_value * (3.3 / 4095.0);  // For a 12-bit ADC, 4095 is the maximum value

    // Store the current value as the last_value for future comparisons or calculations
    last_value = voltage;

    #ifdef DEBUG_VOLTAGE  // If voltage debug mode is enabled
        String debugMsg = "Voltage reading: " + String(voltage) + "V";
        DEBUG_PRINT(DEBUG_VOLTAGE, debugMsg);  // Print the voltage reading to serial for debugging
    #endif

    return voltage;  // Return the measured voltage value
}
