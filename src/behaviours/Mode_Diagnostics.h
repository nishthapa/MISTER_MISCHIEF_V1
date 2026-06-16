#pragma once
#include "behaviours/IRobotMode.h"
#include "core/KinematicsEngine.h"

class Mode_Diagnostics : public IRobotMode {
private:
    KinematicsEngine* kinematics;
    int testState = 0;
    unsigned long stateStartTime = 0;

    // NEW: Memory to hold the user's answers
    int leftMotorTestAns = 0;
    int rightMotorTestAns = 0;

public:
    Mode_Diagnostics(KinematicsEngine* k);
    void onEnter(const GlobalDataBank& robotData) override;
    void update(const RobotMood& currentMood, const GlobalDataBank& robotData) override;
    const char* getName() const override { return "MODE_DIAGNOSTICS"; }
    void onExit() override;
};