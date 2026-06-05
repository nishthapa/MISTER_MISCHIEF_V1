#pragma once
#include "hal/interfaces/I_IMU.h" // Needed for the FusedAngles struct

// The absolute physical truth of the robot at the current millisecond.
struct GlobalSensorState {
    float distanceCM;
    FusedAngles imuAngles;
    bool imuAlive;
};

// The global instance accessible across cores
extern volatile GlobalSensorState CurrentSensorState;