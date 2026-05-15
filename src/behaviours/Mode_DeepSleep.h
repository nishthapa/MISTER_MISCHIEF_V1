#pragma once

#include "behaviours/IRobotMode.h"
#include "hal/XY160D_MotorDriver.h"

class Mode_DeepSleep : public IRobotMode {
private:
    // Needs the motors strictly to force a stop before sleeping
    I_MotorDriver* motors;

public:
    // Dependency Injection constructor
    Mode_DeepSleep(I_MotorDriver* m);

    // The Contract
    void onEnter() override; // The implementation here kills the CPU!
    void update(const RobotMood& currentMood) override;
    const char* getName() const override { return "MODE_DEEP_SLEEP"; }
    void onExit() override {} // Will never be called, CPU resets upon waking
};