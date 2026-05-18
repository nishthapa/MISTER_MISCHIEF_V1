#pragma once

class PIDController {
private:
    float kp, ki, kd;
    float maxIntegral;
    float maxOutput;
    float dFilterCutoffHz;

    // Memory variables
    float integral;
    float previousMeasurement;
    float previousFilteredDTerm;

public:
    // Dependency Injection: Pass the tuning blueprint in when creating the object
    PIDController(float p, float i, float d, float max_i, float max_out, float d_cutoff_hz);

    // The modern Betaflight-style computation engine
    float compute(float setpoint, float measuredValue, float dt);

    public:
    // Allows live updating of PID tunings (call this from the CommandProcessor)
    void setTunings(float p, float i, float d, float max_i, float max_out);

    // Resets the integral and memory (call this if the robot falls over to clear its brain)
    void reset();
};