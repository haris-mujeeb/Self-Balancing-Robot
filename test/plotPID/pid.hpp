#pragma once

#include "Arduino.h"

class PIDController {
  private:
    float integrator;
    float prevError;

    // PID coefficients
    float Kp, Ki, Kd;

    // Output limits
    float IntegrationLimMin, IntegrationLimMax;

  public:
    float Output;
    // Constructor to initialize PID parameters
    PIDController(float _Kp, float _Ki, float _Kd, float _IntegrationLimMin, float _IntegrationLimMax)
      : Kp(_Kp), Ki(_Ki), Kd(_Kd), IntegrationLimMin(_IntegrationLimMin), IntegrationLimMax(_IntegrationLimMax),
        integrator(0.0f), prevError(0.0f), Output(0.0f) {}

    // Method to update the controller with new setpoint and measurement
    float Compute(float setpoint, float measurement);
};
