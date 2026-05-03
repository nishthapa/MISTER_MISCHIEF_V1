#pragma once

namespace TuningHeading {
    // --- The Core Gains ---
    constexpr float KP = 1.2f;  // Proportional: How hard it fights to stay on target
    constexpr float KI = 0.05f; // Integral: How it corrects persistent drift over time
    constexpr float KD = 0.8f;  // Derivative: The dampener that prevents overshooting

    // --- Modern Flight-Controller Limits ---
    // Anti-Windup: Prevents the I-term from building up to infinity if the wheels get stuck
    constexpr float MAX_INTEGRAL = 200.0f; 
    
    // Motor Output Limit (PWM max is usually 255)
    constexpr float MAX_OUTPUT = 255.0f;

    // The Betaflight Secret Weapon: D-Term Low Pass Filter Cutoff Frequency
    // 20Hz means it allows physical movements to pass through, but blocks high-frequency motor vibrations
    constexpr float D_FILTER_CUTOFF_HZ = 20.0f; 
}