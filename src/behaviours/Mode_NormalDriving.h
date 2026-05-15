#pragma once
#include "behaviours/IRobotMode.h"
#include "hal/XY160D_MotorDriver.h"

class Mode_NormalDriving : public IRobotMode {
private:
    I_MotorDriver* motors;

public:
    Mode_NormalDriving(I_MotorDriver* m);
    void onEnter() override;
    void update(const RobotMood& currentMood) override;
    const char* getName() const override { return "MODE_NORMAL_DRIVING"; }
    void onExit() override;
};