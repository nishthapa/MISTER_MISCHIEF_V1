#include "behaviours/Mode_CompassLock.h"
#include "utils/RemoteLogger.h"

Mode_CompassLock::Mode_CompassLock(KinematicsEngine* k) {
    kinematics = k;
    targetYaw = 0.0f;
}

void Mode_CompassLock::onEnter(const volatile GlobalDataBank& robotData) {
    // 1. Take the Snapshot! 
    // The exact moment you pick him up, remember where he is looking.
    targetYaw = robotData.physics.imuAngles.yaw; 
    logger.printf("Compass Lock engaged! Locking heading to: %.1f degrees\n", targetYaw);
}

void Mode_CompassLock::update(const RobotMood& currentMood, const volatile GlobalDataBank& robotData) {
    
    // --- THE HARDWARE SAFETY FIREWALL ---
    // Compass Lock cannot function without an IMU. 
    if (robotData.health.hardwareBitmask & Comms::HealthBit::IMU_OK) {
        
        // IMU is healthy, run the PID loop!
        kinematics->navigateToHeading(targetYaw, robotData.physics.imuAngles.yaw, 0.0f, currentMood.pidAggression);
        
    } else {
        
        // SAFE FALLBACK: The IMU died while we were holding it!
        // Immediately cut the motors to prevent a runaway death spin.
        kinematics->stop();
        // Optional: logger.println("COMPASS LOCK ABORTED: IMU OFFLINE!");
        
    }
}

void Mode_CompassLock::onExit() { 
    kinematics->stop(); 
}