#pragma once

#include "hal/interfaces/I_MotorDriver.h"
#include "core/PIDController.h"
#include "core/GlobalDataBus.h" // For reporting telemetry to the global memory bank

class KinematicsEngine {
private:
    I_MotorDriver* motorDriver;
    PIDController* pointTurnPID;
    PIDController* arcTurnPID;

    // Helper to safely report telemetry
    void reportTelemetry(float left, float right);

    float getShortestAngle(float target, float current);

public:
    KinematicsEngine(I_MotorDriver* m, PIDController* pointPID, PIDController* arcPID);
    
    // The main entry point for closed-loop navigation
    void navigateToHeading(float targetYaw, float currentYaw, float baseSpeed, float moodAggression = 1.0f);
    
    // Direct hardware override for open-loop maneuvers (Maneuvers, sweeps, etc.)
    void rawDrive(float leftSpeed, float rightSpeed);
    
    void stop();
};