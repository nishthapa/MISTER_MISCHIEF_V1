#pragma once
namespace TuningDistanceHold {
    constexpr float KP = 3.5f;  // Gentle push/pull
    constexpr float KI = 0.0f;  // No integral needed for simple distance
    constexpr float KD = 1.0f;  // Dampens the movement so it doesn't overshoot and crash into your hand
    constexpr float MAX_INTEGRAL = 50.0f;
    constexpr float MAX_OUTPUT = 60.0f; // Limit top speed so it doesn't violently ram things
    constexpr float D_FILTER_CUTOFF_HZ = 10.0f; 
}