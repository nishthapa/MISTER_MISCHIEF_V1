#pragma once

#include "behaviours/IRobotMode.h"
#include "core/KinematicsEngine.h"

class Mode_DeepSleep : public IRobotMode {
private:
    // Needed strictly to force a stop before sleeping
    KinematicsEngine* kinematics;

public:
    // Dependency Injection constructor
    Mode_DeepSleep(KinematicsEngine* k);

    // The Contract
    void onEnter(const volatile GlobalSensorState& sensorState) override;
    void update(const RobotMood& currentMood, const volatile GlobalSensorState& sensorState) override;
    const char* getName() const override { return "MODE_DEEP_SLEEP"; }
    void onExit() override {} // Will never be called, CPU resets upon waking
};