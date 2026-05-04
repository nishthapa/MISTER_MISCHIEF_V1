#pragma once
#include "behaviours/RobotMood.h"

// The Contract that every mood or trick must follow
class IRobotMode {
public:
    // Called once when the robot switches INTO this mood
    virtual void onEnter() = 0;

    // Called continuously in the FreeRTOS loop (like 100 times a second)
    // 💥 NEW: Every mode now receives the current mood!
    virtual void update(const RobotMood& currentMood) = 0;

    // Called once when the robot switches OUT OF this mood
    virtual void onExit() = 0;
};