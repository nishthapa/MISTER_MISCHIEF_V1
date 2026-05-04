#pragma once

namespace TuningBalancePlatform {
    // Balancing usually requires a very high Proportional and Derivative gain 
    // to react instantly to gravity before the robot falls over.
    constexpr float KP = 15.0f; 
    constexpr float KI = 0.0f;  // Usually zero or very low for inverted pendulums
    constexpr float KD = 1.5f;  
    constexpr float MAX_INTEGRAL = 100.0f;
    constexpr float MAX_OUTPUT = 100.0f; // Max motor PWM
    constexpr float D_FILTER_CUTOFF_HZ = 15.0f; // Filter out IMU vibration
}