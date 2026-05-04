#pragma once
#include "behaviours/IRobotMode.h"
#include "hal/XY160D_MotorDriver.h"

class Mode_NormalDriving : public IRobotMode {
private:
    XY160D_MotorDriver* motors;

public:
    Mode_NormalDriving(XY160D_MotorDriver* m);
    void onEnter() override;
    void update(const RobotMood& currentMood) override;
    void onExit() override;
};