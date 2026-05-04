#pragma once

#include "behaviours/IRobotMode.h"
#include "hal/MPU6050_IMU.h"
#include "hal/XY160D_MotorDriver.h"
#include "core/PIDController.h"

class Mode_BalancePlatform : public IRobotMode {
private:
    // This mode needs to "see" tilt, "move" motors, and "think" with PID
    MPU6050_IMU* imu;
    XY160D_MotorDriver* motors;
    PIDController* pid;

public:
    // Dependency Injection: Hand the mode its tools
    Mode_BalancePlatform(MPU6050_IMU* i, XY160D_MotorDriver* m, PIDController* p);

    // The Contract
    void onEnter() override;
    void update(const RobotMood& currentMood) override;
    void onExit() override;
};