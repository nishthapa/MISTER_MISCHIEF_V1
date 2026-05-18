#include "config/ConfigurationManager.h"
#include "behaviours/Mode_NormalDriving.h"
// #include "config/PersonalityConfig.h" // Safe to DELETE since we're now pulling these values from the NVS-backed ConfigurationManager
#include "config/SystemConfig.h"
#include "utils/RemoteLogger.h" 
#include <Arduino.h>

Mode_NormalDriving::Mode_NormalDriving(I_MotorDriver* m, I_IMU* i, PIDController* p) {
    motors = m;
    imu = i;
    pid = p;
}

void Mode_NormalDriving::onEnter() {
    logger.println("Mister Mischief is cruising in Normal Driving Mode...");
    
    // Snapshot the exact heading the moment he starts driving
    targetHeading = imu->getAngles().yaw; 
}

float Mode_NormalDriving::getShortestAngle(float target, float current) {
    float delta = target - current;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    return delta;
}

void Mode_NormalDriving::update(const RobotMood& currentMood) {
    float finalSpeed = Config.CRUISING_SPEED * currentMood.speedMultiplier;
    
    // 1. Where are we pointing right now?
    float currentYaw = imu->getAngles().yaw;
    
    // 2. How far off course are we?
    float error = getShortestAngle(targetHeading, currentYaw);
    
    // 3. Ask the PID how hard to steer to fix it
    float dt = SystemConfig::MAIN_LOOP_TICK_RATE_MS / 1000.0f;
    float correction = pid->compute(0.0f, -error, dt);
    
    // 4. Apply mood aggression
    float finalCorrection = correction * currentMood.pidAggression;

    // 5. Mix the steering correction into the base speed
    motors->drive(finalSpeed + finalCorrection, finalSpeed - finalCorrection);
}

void Mode_NormalDriving::onExit() {
    motors->stop();
}