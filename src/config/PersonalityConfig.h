#pragma once

namespace PersonalityConfig {
    // --- Driving Traits ---
    // The default roaming speed (Percentage 0 to 100)
    constexpr float NORMAL_CRUISING_SPEED = 30.0f;

    // --- Spatial Awareness ---
    // How close an object must be (in cm) to trigger a reaction
    constexpr float OBSTACLE_TRIGGER_DISTANCE_CM = 30.0f;

    // --- Emotional Thresholds ---
    // Ticks required to trigger ANGRY mood (1 tick = 10ms in the control loop)
    // 300 ticks = 3.0 seconds of teasing
    constexpr float DISTANCE_HOLD_FRUSTRATION_LIMIT = 300.0f; 
    
    // How fast frustration cools down when left alone
    constexpr float FRUSTRATION_COOLDOWN_RATE = 0.5f; 
    constexpr float FRUSTRATION_HEATUP_RATE = 1.0f;
}