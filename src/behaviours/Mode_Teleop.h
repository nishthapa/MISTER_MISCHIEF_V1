#pragma once

#include "behaviours/IRobotMode.h"
#include "core/KinematicsEngine.h"

class Mode_Teleop : public IRobotMode {
private:
    KinematicsEngine* kinematics;
    float targetHeading; // Memory for PID drive

    // For the EMA filter for joystick smoothing to avoid derivative kick on sudden stick movements
    float smoothedX;
    float smoothedY;

public:
    Mode_Teleop(KinematicsEngine* k);
    
    void onEnter(const GlobalDataBank& robotData) override;
    void update(const RobotMood& currentMood, const GlobalDataBank& robotData) override;
    const char* getName() const override { return "MODE_TELEOP"; }
    void onExit() override;
};