#include "pid.hpp"

// Method to update the controller with new setpoint and measurement
float PIDController::Compute(float setpoint, float measurement) {
  // Error signal
  float error = setpoint - measurement;

  // Proportional term
  float proportional = Kp * error;

  // Differentional term
  float differentiator = Kd * (error - prevError);

  // Integral term with trapezoidal rule
  integrator += 0.5f * Ki * (error + prevError); // ?? prevError remove ?

  // Anti-windup via dynamic integrator clamping
  float limMaxInt = (IntegrationLimMax > proportional) ? IntegrationLimMax - proportional : 0.0f;
  float limMinInt = (IntegrationLimMin < proportional) ? IntegrationLimMin - proportional : 0.0f;

  // Clamp Integrator
  integrator = constrain(integrator, limMinInt, limMaxInt);

  // Store current error and measurement for the next update
  prevError = error;

  // Compute the output
  Output = proportional + integrator + differentiator;
  return  Output;
}