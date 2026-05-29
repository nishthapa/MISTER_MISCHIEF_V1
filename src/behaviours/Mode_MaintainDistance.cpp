#include "behaviours/Mode_MaintainDistance.h"
#include "utils/RemoteLogger.h"
// #include "config/PersonalityConfig.h" // Safe to DELETE since we're now pulling these values from the NVS-backed ConfigurationManager
#include "config/SystemConfig.h"      // <-- For the master clock
#include "config/ConfigurationManager.h"

Mode_MaintainDistance::Mode_MaintainDistance(I_DistanceSensor* s, KinematicsEngine* k, PIDController* p) {
    distSensor = s; kinematics = k; pid = p;
}

void Mode_MaintainDistance::onEnter() {
    logger.println("Mister Mischief is maintaining distance!");
}

void Mode_MaintainDistance::update(const RobotMood& currentMood) {
    float currentDistance = distSensor->getDistanceCM();
    
    // Calculate dt (Delta Time) in seconds dynamically from the Master Clock!
    float dt = SystemConfig::MAIN_LOOP_TICK_RATE_MS / 1000.0f;
    
    // Target distance to maintain is defined in PersonalityConfig
    // float correction = pid->compute(PersonalityConfig::MAINTAIN_DISTANCE_TARGET_CM, currentDistance, dt); 
    float correction = pid->compute(Config.MAINTAIN_DISTANCE_CM, currentDistance, dt);
    float finalSpeed = correction * currentMood.speedMultiplier;
    kinematics->rawDrive(-finalSpeed, -finalSpeed);
}