#include "core/KinematicsEngine.h"
#include "config/SystemConfig.h" 

// 1. Constructor Updated
KinematicsEngine::KinematicsEngine(PIDController* pointPID, PIDController* arcPID) {
    pointTurnPID = pointPID;
    arcTurnPID = arcPID;
}

// 2. Implement the telemetry reporter
// void KinematicsEngine::reportTelemetry(float left, float right) {
//     //portENTER_CRITICAL(&globalDataBusLock); // TODO: We might want to add a critical section
//     // Cast float PWM to int16_t for the telemetry bus
//     CurrentRobotData.physics.leftMotorPWM = static_cast<int16_t>(left);
//     CurrentRobotData.physics.rightMotorPWM = static_cast<int16_t>(right);
//     //portEXIT_CRITICAL(&globalDataBusLock);  // TODO: end the critical section
// }

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
}

// 3. Raw Drive Updated
void KinematicsEngine::rawDrive(float leftSpeed, float rightSpeed) {
    outTargetHeading = 0.0f;
    outHeadingError = 0.0f;
    outLeftPWM = static_cast<int16_t>(leftSpeed);
    outRightPWM = static_cast<int16_t>(rightSpeed);
}

// 4. Stop Updated
void KinematicsEngine::stop() {
    outTargetHeading = 0.0f;
    outHeadingError = 0.0f;
    outLeftPWM = 0;
    outRightPWM = 0;
}