#include "Arduino.h"
#include <stdint.h>
#include "pins.hpp"
#include "debugConfig.h"

extern float last_value;

void voltage_init();
float voltage_read();