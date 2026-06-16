#pragma once

#include <Arduino.h>
#include "core/BehaviourEngine.h"
#include "hal/interfaces/I_MotorDriver.h"
#include "config/ConfigurationManager.h" // Needed for SysConfig
#include "core/KinematicsEngine.h" // For instructing the Motor Driver to follow the GlobalDataBus Actuation values
#include "core/PIDController.h" // Add this include

// 1. Define the minimal Context Struct
struct ControlLoopContext {
    BehaviourEngine* brain;
    I_MotorDriver* motorDriver;
    KinematicsEngine* kinematics;
    PIDController* distancePID; // <--- NEW: Add the missing distance PID!
};

// 2. The FreeRTOS task signature
void ControlLoopTask(void *pvParameters);