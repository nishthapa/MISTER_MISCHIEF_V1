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

    bool isDriving                      = false;
    bool isDrivingStraightForward       = false;
    bool isDrivingStraightReverse       = false;
    bool isPointTurningLeft             = false;
    bool isPointTurningRight            = false;
    bool isArcTurningForwardLeft        = false;
    bool isArcTurningForwardRight       = false;
    bool isArcTurningReverseLeft        = false;
    bool isArcTurningReverseRight       = false;
    bool isOneSidedTurningForwardLeft   = false;
    bool isOneSidedTurningForwardRight  = false;
    bool isOneSidedTurningReverseLeft   = false;
    bool isOneSidedTurningReverseRight  = false;

    float outTargetHeading = 0.0f;
    float outHeadingError = 0.0f;

    float getShortestAngle(float target, float current);
    void updateMovementFlags(); // No arguments needed since it internal reads class variables

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

    bool getIsDriving() const { return isDriving; }
    bool getIsDrivingStraightForward() const { return isDrivingStraightForward; } // Fixed
    bool getIsDrivingStraightReverse() const { return isDrivingStraightReverse; } // Fixed
    bool getIsPointTurningLeft() const { return isPointTurningLeft; }             // Fixed
    bool getIsPointTurningRight() const { return isPointTurningRight; }           // Fixed
    bool getIsArcTurningForwardLeft() const { return isArcTurningForwardLeft; }   // Fixed
    bool getIsArcTurningForwardRight() const { return isArcTurningForwardRight; } // Fixed
    bool getIsArcTurningReverseLeft() const { return isArcTurningReverseLeft; }   // Fixed
    bool getIsArcTurningReverseRight() const { return isArcTurningReverseRight; } // Fixed
    bool getIsOneSidedTurningForwardLeft() const { return isOneSidedTurningForwardLeft; }
    bool getIsOneSidedTurningForwardRight() const { return isOneSidedTurningForwardRight; }
    bool getIsOneSidedTurningReverseLeft() const { return isOneSidedTurningReverseLeft; }
    bool getIsOneSidedTurningReverseRight() const { return isOneSidedTurningReverseRight; }



    float getTargetHeading() const { return outTargetHeading; }
    float getHeadingError() const { return outHeadingError; }
};