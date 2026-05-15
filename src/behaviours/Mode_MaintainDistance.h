#pragma once

#include "behaviours/IRobotMode.h"
#include "hal/HCSR04_Sonar.h"
#include "hal/XY160D_MotorDriver.h"
#include "core/PIDController.h"

class Mode_MaintainDistance : public IRobotMode {
private:
    // The tools this mode needs to borrow
    I_DistanceSensor* sonar;
    I_MotorDriver* motors;
    PIDController* pid;

public:
    // Dependency Injection constructor
    Mode_MaintainDistance(I_DistanceSensor* s, I_MotorDriver* m, PIDController* p);

    // The Contract
    void onEnter() override; // Nothing special needed on boot
    void update(const RobotMood& currentMood) override;
    const char* getName() const override { return "MODE_MAINTAIN_DISTANCE"; }
    void onExit() override { motors->stop(); } // Safety stop when leaving this mode
};