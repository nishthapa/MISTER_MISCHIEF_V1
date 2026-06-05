#pragma once
#include "behaviours/IRobotMode.h"
#include "core/KinematicsEngine.h"

class Mode_NormalDriving : public IRobotMode {
private:
    KinematicsEngine* kinematics; // ONLY kinematics, NO IMU! Handled by the GlobalSensorState now!
    float targetHeading; 
    float getShortestAngle(float target, float current);

public:
    Mode_NormalDriving(KinematicsEngine* k);
    
    void onEnter(const volatile GlobalSensorState& sensorState) override;
    void update(const RobotMood& currentMood, const volatile GlobalSensorState& sensorState) override;
    const char* getName() const override { return "MODE_NORMAL_DRIVING"; }
    void onExit() override;
};