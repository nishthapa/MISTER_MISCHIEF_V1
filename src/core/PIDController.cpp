#include "core/PIDController.h"
#include <math.h>

PIDController::PIDController(float p, float i, float d, float max_i, float max_out, float d_cutoff_hz) {
    kp = p; 
    ki = i; 
    kd = d;
    maxIntegral = max_i; 
    maxOutput = max_out;
    dFilterCutoffHz = d_cutoff_hz;

    reset(); // Initialize memory to zero
}

void PIDController::reset() {
    integral = 0.0f;
    previousMeasurement = 0.0f;
    previousFilteredDTerm = 0.0f;
}

float PIDController::compute(float setpoint, float measuredValue, float dt) {
    // Safety check to prevent divide-by-zero if loop runs too fast
    if (dt <= 0.0f) return 0.0f; 

    float error = setpoint - measuredValue;

    // ---------------------------------------------------------
    // 1. PROPORTIONAL (The Spring)
    // ---------------------------------------------------------
    float pTerm = kp * error;

    // ---------------------------------------------------------
    // 2. INTEGRAL (The Patience with Anti-Windup)
    // ---------------------------------------------------------
    integral += ki * error * dt;

    // Clamp the integral to prevent runaway windup
    if (integral > maxIntegral) integral = maxIntegral;
    else if (integral < -maxIntegral) integral = -maxIntegral;

    // ---------------------------------------------------------
    // 3. DERIVATIVE (The Shock Absorber - Betaflight Style)
    // ---------------------------------------------------------
    // Calculate based on measurement change, not error change (Avoids Derivative Kick)
    // We negate it because D should fight the direction of the physical movement
    float measurementDelta = (measuredValue - previousMeasurement) / dt;
    float rawDTerm = -kd * measurementDelta;
    previousMeasurement = measuredValue;

    // Apply PT1 Low-Pass Filter to silence sensor noise
    float rc = 1.0f / (2.0f * M_PI * dFilterCutoffHz);
    float alpha = dt / (rc + dt);
    
    float filteredDTerm = previousFilteredDTerm + alpha * (rawDTerm - previousFilteredDTerm);
    previousFilteredDTerm = filteredDTerm;

    // ---------------------------------------------------------
    // 4. OUTPUT SUMMATION & SATURATION
    // ---------------------------------------------------------
    float output = pTerm + integral + filteredDTerm;

    // Clamp the final output to the physical limits of the motor driver
    if (output > maxOutput) output = maxOutput;
    else if (output < -maxOutput) output = -maxOutput;

    return output;
}