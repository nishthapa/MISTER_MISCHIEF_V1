#pragma once

#include "behaviours/IRobotMode.h"
#include "hal/XY160D_MotorDriver.h"

class Mode_Dizzy : public IRobotMode {
private:
    // This mode acts blind, it only needs the muscles
    XY160D_MotorDriver* motors;

public:
    // Dependency Injection constructor
    Mode_Dizzy(XY160D_MotorDriver* m);

    // The Contract
    void onEnter() override;
    void update(const RobotMood& currentMood) override;
    void onExit() override { motors->stop(); } // Safety stop when he shakes the dizziness off
};