#include "core/KinematicsEngine.h"
#include "config/SystemConfig.h" 

KinematicsEngine::KinematicsEngine(I_MotorDriver* m, PIDController* pointPID, PIDController* arcPID) {
    motorDriver = m;
    pointTurnPID = pointPID;
    arcTurnPID = arcPID;
}

float KinematicsEngine::getShortestAngle(float target, float current) {
    float delta = target - current;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    return delta;
}

void KinematicsEngine::navigateToHeading(float targetYaw, float currentYaw, float baseSpeed, float moodAggression) {
    float error = getShortestAngle(targetYaw, currentYaw);
    float dt = SystemConfig::MAIN_LOOP_TICK_RATE_MS / 1000.0f;
    float steeringCorrection = 0.0f;

    // Kinematic Profiler: Choose PID profile based on state
    if (baseSpeed == 0.0f) {
        // Stationary: Apply high proportional authority to break static scrub friction
        steeringCorrection = pointTurnPID->compute(0.0f, -error, dt);
    } else {
        // Rolling: Apply damped heading-hold adjustments for low dynamic friction
        steeringCorrection = arcTurnPID->compute(0.0f, -error, dt);
    }
    
    steeringCorrection *= moodAggression;

    // Kinematic Mixer
    float leftPWM = baseSpeed + steeringCorrection;
    float rightPWM = baseSpeed - steeringCorrection;
    
    motorDriver->drive(leftPWM, rightPWM);
}

void KinematicsEngine::rawDrive(float leftSpeed, float rightSpeed) {
    motorDriver->drive(leftSpeed, rightSpeed);
}

void KinematicsEngine::stop() {
    motorDriver->stop();
}