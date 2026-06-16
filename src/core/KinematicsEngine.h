#pragma once

#include <Arduino.h>
#include "core/PIDController.h"

class KinematicsEngine {
private:
    PIDController* pointTurnPID;
    PIDController* arcTurnPID;

    // NEW: Memory to hold the final mathematical answers
    int16_t outLeftPWM = 0;
    int16_t outRightPWM = 0;
    float outTargetHeading = 0.0f;
    float outHeadingError = 0.0f;

    float getShortestAngle(float target, float current);

public:
    // Removed the I_MotorDriver!
    KinematicsEngine(PIDController* pointPID, PIDController* arcPID);
    
    void reloadPIDTunings();
    void navigateToHeading(float targetYaw, float currentYaw, float baseSpeed, float moodAggression = 1.0f);
    void rawDrive(float leftSpeed, float rightSpeed);
    void stop();

    // NEW: Getters for the Task to pull the answers
    int16_t getLeftPWM() const { return outLeftPWM; }
    int16_t getRightPWM() const { return outRightPWM; }
    float getTargetHeading() const { return outTargetHeading; }
    float getHeadingError() const { return outHeadingError; }
};