#pragma once
#include "behaviours/IRobotMode.h"
#include "hal/interfaces/I_IMU.h"
#include "core/KinematicsEngine.h" // <-- ADDED THIS

class Mode_CompassLock : public IRobotMode {
private:
    I_IMU* imu;
    KinematicsEngine* kinematics; // <-- REPLACED MOTORS AND PID WITH THIS
    float targetYaw;

public:
    // Dependency injection: Hand the Mode its tools (IMU and Kinematics Engine)
    Mode_CompassLock(I_IMU* i, KinematicsEngine* k);

    void onEnter() override;
    void update(const RobotMood& currentMood) override;
    const char* getName() const override { return "MODE_COMPASS_LOCK"; }
    void onExit() override;
};