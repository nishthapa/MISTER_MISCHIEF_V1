#pragma once
#include "behaviours/IRobotMode.h"
#include "hal/interfaces/I_MotorDriver.h"
#include "hal/interfaces/I_IMU.h"
#include "core/PIDController.h"

class Mode_NormalDriving : public IRobotMode {
private:
    I_MotorDriver* motors;
    I_IMU* imu;
    PIDController* pid;

    float targetHeading; // The heading we want to lock onto

    // Helper to prevent the robot from spinning the wrong way across the 360-degree boundary
    float getShortestAngle(float target, float current);

public:
    // The upgraded constructor!
    Mode_NormalDriving(I_MotorDriver* m, I_IMU* i, PIDController* p);
    
    void onEnter() override;
    void update(const RobotMood& currentMood) override;
    const char* getName() const override { return "MODE_NORMAL_DRIVING"; }
    void onExit() override;
};