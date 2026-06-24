// src/core/interfaces/IBehaviourDecider.h
#pragma once

#include "core/GlobalDataBus.h"
#include "behaviours/IRobotMode.h"
#include "behaviours/RobotMood.h"

class IBehaviourDecider {
public:
    virtual ~IBehaviourDecider() = default;
    
    // Evaluates sensor data and dictates the next mode and mood
    virtual void evaluate(const GlobalDataBank& dataBus, SystemMode currentMode, SystemMode& outRequestedMode, RobotMood& outMood) = 0;
};