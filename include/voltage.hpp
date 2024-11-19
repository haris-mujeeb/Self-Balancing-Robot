#include "Arduino.h"
#include <stdint.h>
#include "pins.hpp"
#include "debugConfig.h"


void voltage_init() {
  analogReference(INTERNAL);
}

float voltage_read() {
  static unsigned long voltage_measure_time = 0;  // Note: initialized only once due to static
  static float last_value = 0.0;
  if (millis() - voltage_measure_time > 1000) {
    voltage_measure_time = millis();
    last_value = (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5);
    String debugMsg = String(voltage_measure_time) +  "] [value: " + String(last_value)+"]";
    DEBUG_PRINT(DEBUG_VOLTAGE, debugMsg);
  }
  return last_value;
}