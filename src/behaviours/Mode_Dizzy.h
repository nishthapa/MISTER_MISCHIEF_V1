#pragma once

#include "behaviours/IRobotMode.h"
#include "core/KinematicsEngine.h"

class Mode_Dizzy : public IRobotMode {
private:
    KinematicsEngine* kinematics; // Upgraded to Kinematics Engine for uniform abstraction

public:
    Mode_Dizzy(KinematicsEngine* k);

    void onEnter(const GlobalDataBank& robotData) override;
    void update(const RobotMood& currentMood, const GlobalDataBank& robotData) override;
    const char* getName() const override { return "MODE_DIZZY"; }
    void onExit() override { kinematics->stop(); } 
};