#pragma once

#include "behaviours/IRobotMode.h"
#include "core/KinematicsEngine.h"
#include "core/PIDController.h"

class Mode_MaintainDistance : public IRobotMode {
private:
    KinematicsEngine* kinematics; // NO SONAR! Handled by the RobotData now
    PIDController* pid;

public:
    Mode_MaintainDistance(KinematicsEngine* k, PIDController* p);
    
    void onEnter(const volatile GlobalDataBank& robotData) override;
    void update(const RobotMood& currentMood, const volatile GlobalDataBank& robotData) override;
    const char* getName() const override { return "MODE_MAINTAIN_DISTANCE"; }
    void onExit() override;
};