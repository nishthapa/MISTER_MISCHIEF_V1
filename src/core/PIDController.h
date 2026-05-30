#pragma once

class PIDController {
private:
    float kp, ki, kd;
    float maxIntegral;
    float maxOutput;

    float deadband; // <--- NEW!

    float dFilterCutoffHz;

    // Memory variables
    float integral;
    float previousMeasurement;
    float previousFilteredDTerm;

public:
    // Insert dead_zone as the 6th param. Default the 7th param to 15Hz to keep the factory clean!
    PIDController(float p, float i, float d, float max_i, float max_out, float dead_zone, float d_cutoff_hz = 15.0f);

    // The modern Betaflight-style computation engine
    float compute(float setpoint, float measuredValue, float dt);

    public:
    // Allows live updating of PID tunings (call this from the CommandProcessor)
    void setTunings(float p, float i, float d, float max_i, float max_out);

    // Resets the integral and memory (call this if the robot falls over to clear its brain)
    void reset();
};