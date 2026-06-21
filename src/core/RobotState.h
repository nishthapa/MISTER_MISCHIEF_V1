#pragma once

#include <stdint.h>

enum class SystemMode : uint8_t {
    // --- OS STATES ---
    BOOTING = 0,
    MANUAL_OVERRIDE = 1,

    // --- BEHAVIOURAL MODES ---
    MODE_NORMAL_DRIVING = 2,
    MODE_OBSTACLE_AVOIDANCE = 3,
    MODE_MAINTAIN_DISTANCE = 4,
    MODE_COMPASS_LOCK = 5,
    MODE_DIZZY = 6,
    MODE_DEEP_SLEEP = 7,
    MODE_AUTOTUNE = 8,
    MODE_DIAGNOSTICS = 9
};

// The global runtime variable
// Not volatile anymore and centralized in GlobalDataBus
// extern volatile SystemMode GLOBAL_MODE; 