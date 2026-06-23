#include "core/KinematicsEngine.h"
#include "config/SystemConfig.h"
#include "config/ConfigurationManager.h"

// 1. Constructor Updated
KinematicsEngine::KinematicsEngine(PIDController* pointPID, PIDController* arcPID) {
    pointTurnPID = pointPID;
    arcTurnPID = arcPID;
}

void KinematicsEngine::reloadPIDTunings() {
    // 1. Unified sync of the Point Turn PID
    pointTurnPID->setTunings(
        SysConfig.PID_POINT_P, SysConfig.PID_POINT_I, SysConfig.PID_POINT_D,
        SysConfig.PID_POINT_ILIM, SysConfig.PID_POINT_LIM
    );

    // 2. Unified sync of the Arc Turn PID
    arcTurnPID->setTunings(
        SysConfig.PID_ARC_P, SysConfig.PID_ARC_I, SysConfig.PID_ARC_D,
        SysConfig.PID_ARC_ILIM, SysConfig.PID_ARC_LIM
    );
}

float KinematicsEngine::getShortestAngle(float target, float current) {
    float delta = target - current;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    return delta;
}

// 2. Navigation Updated
void KinematicsEngine::navigateToHeading(float targetYaw, float currentYaw, float baseSpeed, float moodAggression) {
    float error = getShortestAngle(targetYaw, currentYaw);
    float dt = SystemConfig::MAIN_LOOP_TICK_RATE_MS / 1000.0f;
    float steeringCorrection = 0.0f;

    // Kinematic Profiler: Choose PID profile based on state
    if (baseSpeed == 0.0f) {
        // Stationary: Apply high proportional authority to break static scrub friction
        
        // ADDED THE MINUS SIGN BACK SINCE YAW DIRECTION NOW FOLLOWS ESTABLISHED CONVENTIONS!
        steeringCorrection = pointTurnPID->compute(0.0f, -error, dt); 
    } else {
        // Rolling: Apply damped heading-hold adjustments for low dynamic friction
        
        // ADDED THE MINUS SIGN BACK SINCE YAW DIRECTION NOW FOLLOWS ESTABLISHED CONVENTIONS!
        steeringCorrection = arcTurnPID->compute(0.0f, -error, dt);   
    }
    
    steeringCorrection *= moodAggression;

    // Kinematic Mixer
    float leftPWM = baseSpeed + steeringCorrection;
    float rightPWM = baseSpeed - steeringCorrection;
    
    //motorDriver->drive(leftPWM, rightPWM);

    // 3. Update telemetry immediately after command
    //reportTelemetry(leftPWM, rightPWM);

    // SAVE THE ANSWERS, DO NOT TOUCH HARDWARE
    outTargetHeading = targetYaw;
    outHeadingError = error;
    outLeftPWM = static_cast<int16_t>(leftPWM);
    outRightPWM = static_cast<int16_t>(rightPWM);
    if (outLeftPWM || outRightPWM) {
        isDriving = true;
    } else {
        isDriving = false;
    }
}

// 3. Raw Drive Updated
void KinematicsEngine::rawDrive(float leftSpeed, float rightSpeed) {
    outTargetHeading = 0.0f;
    outHeadingError = 0.0f;
    outLeftPWM = static_cast<int16_t>(leftSpeed);
    outRightPWM = static_cast<int16_t>(rightSpeed);
    if (outLeftPWM || outRightPWM) {
        isDriving = true;
    } else {
        isDriving = false;
    }
}

// 4. Stop Updated
void KinematicsEngine::stop() {
    outTargetHeading = 0.0f;
    outHeadingError = 0.0f;
    outLeftPWM = 0;
    outRightPWM = 0;
    isDriving = false;
}