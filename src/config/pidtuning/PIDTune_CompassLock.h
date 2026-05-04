#pragma once

namespace TuningCompassLock {
    // Compass lock (Yaw rotation) requires much gentler tuning than balancing.
    constexpr float KP = 2.5f;   // Gentle rotational push
    constexpr float KI = 0.1f;   // Small integral to overcome static track friction if he gets "stuck" near the target
    constexpr float KD = 0.5f;   // Dampens the rotation as he approaches the target heading
    constexpr float MAX_INTEGRAL = 30.0f;
    constexpr float MAX_OUTPUT = 80.0f; // Fast, but not maximum violently fast
    constexpr float D_FILTER_CUTOFF_HZ = 10.0f;
}