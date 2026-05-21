#pragma once

#include "behaviours/IRobotMode.h"
#include "hal/hardware/distancesensor/HCSR04_Sonar.h"
#include "core/KinematicsEngine.h" //For the new kinematics engine
#include "core/PIDController.h"

class Mode_MaintainDistance : public IRobotMode {
private:
    // The tools this mode needs to borrow
    I_DistanceSensor* sonar;
    KinematicsEngine* kinematics;
    PIDController* pid; // Exclusively needed directly (instead of through the Kinematics Engine) for maintaining distance to a hand

public:
    // Dependency Injection constructor
    Mode_MaintainDistance(I_DistanceSensor* s, KinematicsEngine* k, PIDController* p);

    // The Contract
    void onEnter() override; // Nothing special needed on boot
    void update(const RobotMood& currentMood) override;
    const char* getName() const override { return "MODE_MAINTAIN_DISTANCE"; }
    void onExit() override { kinematics->stop(); } // Safety stop when leaving this mode
};