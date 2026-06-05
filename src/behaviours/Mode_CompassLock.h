#pragma once
#include "behaviours/IRobotMode.h"
#include "core/KinematicsEngine.h" // <-- ADDED THIS

class Mode_CompassLock : public IRobotMode {
private:
    KinematicsEngine* kinematics; // ONLY kinematics, NO IMU!
    float targetYaw;

public:
    // Dependency injection: Hand the Mode its tools (Kinematics Engine)
    Mode_CompassLock(KinematicsEngine* k);

    void onEnter(const volatile GlobalSensorState& sensorState) override;
    void update(const RobotMood& currentMood, const volatile GlobalSensorState& sensorState) override;
    const char* getName() const override { return "MODE_COMPASS_LOCK"; }
    void onExit() override;
};