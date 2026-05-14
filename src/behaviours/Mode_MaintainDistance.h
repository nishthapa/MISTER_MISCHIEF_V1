#pragma once

#include "behaviours/IRobotMode.h"
#include "hal/HCSR04_Sonar.h"
#include "hal/XY160D_MotorDriver.h"
#include "core/PIDController.h"

class Mode_MaintainDistance : public IRobotMode {
private:
    // The tools this mode needs to borrow
    HCSR04_Sonar* sonar;
    XY160D_MotorDriver* motors;
    PIDController* pid;

public:
    // Dependency Injection constructor
    Mode_MaintainDistance(HCSR04_Sonar* s, XY160D_MotorDriver* m, PIDController* p);

    // The Contract
    void onEnter() override; // Nothing special needed on boot
    void update(const RobotMood& currentMood) override;
    void onExit() override { motors->stop(); } // Safety stop when leaving this mode
};