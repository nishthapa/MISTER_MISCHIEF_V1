#pragma once

enum class SystemMode {
    // --- OS STATES ---
    BOOTING,
    MANUAL_OVERRIDE,

    // --- BEHAVIOURAL MODES ---
    MODE_NORMAL_DRIVING,
    MODE_OBSTACLE_AVOIDANCE,
    MODE_MAINTAIN_DISTANCE,
    MODE_COMPASS_LOCK,
    MODE_DIZZY,
    MODE_DEEP_SLEEP,
    MODE_AUTOTUNE
};

// The global runtime variable
extern volatile SystemMode GLOBAL_MODE;