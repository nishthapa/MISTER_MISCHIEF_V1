#pragma once

#include "behaviours/IRobotMode.h"
#include "hal/I_IMU.h"
#include "hal/XY160D_MotorDriver.h"
#include "core/PIDController.h"

class Mode_CompassLock : public IRobotMode {
private:
    // This mode needs to "see" tilt, "move" motors, and "think" with PID
    I_IMU* imu;
    I_MotorDriver* motors;
    PIDController* pid;

    // The memory of where he was looking when he was picked up
    float targetYaw;

public:
    // Dependency Injection: Hand the mode its tools
    Mode_CompassLock(I_IMU* i, I_MotorDriver* m, PIDController* p);

    // The Contract
    void onEnter() override;
    void update(const RobotMood& currentMood) override;
    const char* getName() const override { return "MODE_COMPASS_LOCK"; }
    void onExit() override;
};