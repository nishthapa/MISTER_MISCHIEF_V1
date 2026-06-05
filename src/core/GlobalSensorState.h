#pragma once
#include "hal/interfaces/I_IMU.h"

// 1. DATA: The absolute physical truth of the robot (Hardware -> Brain)
struct GlobalSensorState {
    float distanceCM;
    FusedAngles imuAngles;
    bool imuAlive;
};

// 2. COMMANDS: Hardware requests (Brain -> Hardware)
struct HardwareCommandBus {
    bool requestGyroCalibration;
    bool requestAccelCalibration;
    bool requestMagCalibration;
    float targetFilterBeta;
};

// The global instances accessible across cores
extern volatile GlobalSensorState CurrentSensorState;
extern volatile HardwareCommandBus HardwareCommands;