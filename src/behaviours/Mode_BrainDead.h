#pragma once
#include "IRobotMode.h"
#include "core/KinematicsEngine.h"

class Mode_BrainDead : public virtual IRobotMode {
private:
    KinematicsEngine* kinematics;
public:
    Mode_BrainDead(KinematicsEngine* kinEngine);
    void onEnter(const GlobalDataBank& robotData) override;
    // FIX: Added 'const' and '&' to match IRobotMode precisely
    void update(const RobotMood& currentMood, const GlobalDataBank& robotData) override;
    void onExit() override;
    const char* getName() const override { return "MODE_BRAIN_DEAD"; }
};