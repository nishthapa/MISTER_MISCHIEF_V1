#include "core/KinematicsEngine.h"
#include "config/SystemConfig.h" 

KinematicsEngine::KinematicsEngine(I_MotorDriver* m, PIDController* pointPID, PIDController* arcPID) {
    motorDriver = m;
    pointTurnPID = pointPID;
    arcTurnPID = arcPID;
}

// 2. Implement the telemetry reporter
void KinematicsEngine::reportTelemetry(float left, float right) {
    //portENTER_CRITICAL(&globalDataBusLock); // TODO: We might want to add a critical section
    // Cast float PWM to int16_t for the telemetry bus
    CurrentRobotData.physics.leftMotorPWM = static_cast<int16_t>(left);
    CurrentRobotData.physics.rightMotorPWM = static_cast<int16_t>(right);
    //portEXIT_CRITICAL(&globalDataBusLock);  // TODO: end the critical section
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
    
    motorDriver->drive(leftPWM, rightPWM);

    // 3. Update telemetry immediately after command
    reportTelemetry(leftPWM, rightPWM);
}

void KinematicsEngine::rawDrive(float leftSpeed, float rightSpeed) {
    motorDriver->drive(leftSpeed, rightSpeed);

    // 3. Update telemetry immediately after command
    reportTelemetry(leftSpeed, rightSpeed);
}

void KinematicsEngine::stop() {
    motorDriver->stop();

    // 4. Update telemetry to zero
    reportTelemetry(0.0f, 0.0f);
}