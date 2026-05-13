#pragma once

namespace TuningObstacleAvoidanceNewPathScanSweep {
    // Needs to be snappy to quickly face the escape vector, but not so aggressive it overshoots
    constexpr float KP = 3.0f;   
    constexpr float KI = 0.1f;   // Small integral to overcome static track friction 
    constexpr float KD = 0.6f;   // Dampener to prevent overshooting the target angle
    constexpr float MAX_INTEGRAL = 30.0f;
    constexpr float MAX_OUTPUT = 85.0f; // Fast alignment speed
    constexpr float D_FILTER_CUTOFF_HZ = 10.0f;
}