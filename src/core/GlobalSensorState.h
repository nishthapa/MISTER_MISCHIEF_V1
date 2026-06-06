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

// 3. TELEOPERATION: For Android phone remote control
struct TeleopCommandBus {
    float joyX;       // -1.0f (Left) to 1.0f (Right)
    float joyY;       // -1.0f (Reverse) to 1.0f (Forward)
    bool usePIDDrive; // Toggles between RAW motor mixing and PID Heading Hold
    bool isConnected; // Failsafe: Stops robot if BLE drops
};

// The global instances accessible across cores
extern volatile GlobalSensorState CurrentSensorState;
extern volatile HardwareCommandBus HardwareCommands;
extern volatile TeleopCommandBus TeleopCommands;