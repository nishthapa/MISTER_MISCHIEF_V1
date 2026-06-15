#pragma once

#include <Arduino.h>
#include "core/BehaviourEngine.h"
#include "hal/interfaces/I_MotorDriver.h"
#include "config/ConfigurationManager.h" // Needed for SysConfig

// 1. Define the minimal Context Struct
struct ControlLoopContext {
    BehaviourEngine* brain;
    I_MotorDriver* motorDriver;
    // Removed teleopMode and normalMode! as mode switching
    // is done by BehaviourEngine
};

// 2. The FreeRTOS task signature
void ControlLoopTask(void *pvParameters);