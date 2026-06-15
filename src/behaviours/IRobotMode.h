#pragma once
#include "behaviours/RobotMood.h"
#include "core/GlobalDataBus.h" // <--- Include the memory struct

// The Contract that every mood or trick must follow
class IRobotMode {
public:
    // Called once when the robot switches INTO this mood
    virtual void onEnter(const GlobalDataBank& robotData) = 0; // <-- Added state

    // Called continuously in the FreeRTOS loop (like 100 times a second)
    // 💥 NEW: Every mode now receives the current mood!
    virtual void update(const RobotMood& currentMood, const GlobalDataBank& robotData) = 0; // <-- Added state

    // NEW: Force every mode to return its name!
    virtual const char* getName() const = 0;

    // Called once when the robot switches OUT OF this mood
    virtual void onExit() = 0;    
};