#include "config/ConfigurationManager.h"
#include "behaviours/Mode_NormalDriving.h"
#include "utils/RemoteLogger.h" 

Mode_NormalDriving::Mode_NormalDriving(I_IMU* i, KinematicsEngine* k) {
    imu = i; kinematics = k;
    targetHeading = 0.0f;
}

void Mode_NormalDriving::onEnter() {
    logger.println("Mister Mischief is cruising in Normal Driving Mode...");
    
    // Snapshot the exact heading the moment he starts driving
    targetHeading = imu->getAngles().yaw; 
}

void Mode_NormalDriving::update(const RobotMood& currentMood) {
    float finalSpeed = Config.CRUISING_SPEED * currentMood.speedMultiplier;
    // Engine automatically routes to Arc Turn profile because base speed > 0
    kinematics->navigateToHeading(targetHeading, imu->getAngles().yaw, finalSpeed, currentMood.pidAggression);
}

void Mode_NormalDriving::onExit() { kinematics->stop(); }