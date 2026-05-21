#pragma once
#include "behaviours/IRobotMode.h"
#include "hal/interfaces/I_IMU.h"
#include "core/KinematicsEngine.h" // <-- ADDED THIS

class Mode_NormalDriving : public IRobotMode {
private:
    I_IMU* imu;
    KinematicsEngine* kinematics; // <-- REPLACED MOTORS AND PID WITH THIS

    float targetHeading; 
    float getShortestAngle(float target, float current);

public:
    // Dependency injection: Hand the Mode its tools (IMU and Kinematics Engine)
    Mode_NormalDriving(I_IMU* i, KinematicsEngine* k);
    
    void onEnter() override;
    void update(const RobotMood& currentMood) override;
    const char* getName() const override { return "MODE_NORMAL_DRIVING"; }
    void onExit() override;
};