#include "config/ConfigurationManager.h"
#include "behaviours/Mode_NormalDriving.h"
#include "utils/RemoteLogger.h" 

Mode_NormalDriving::Mode_NormalDriving(KinematicsEngine* k) {
    kinematics = k;
    targetHeading = 0.0f;
}

void Mode_NormalDriving::onEnter(const volatile GlobalSensorState& sensorState) {
    logger.println("Mister Mischief is cruising in Normal Driving Mode...");
    targetHeading = sensorState.imuAngles.yaw; // Read from Memory!
}

void Mode_NormalDriving::update(const RobotMood& currentMood, const volatile GlobalSensorState& sensorState) {
    float finalSpeed = SysConfig.CRUISING_SPEED * currentMood.speedMultiplier;
    kinematics->navigateToHeading(targetHeading, sensorState.imuAngles.yaw, finalSpeed, currentMood.pidAggression);
}

void Mode_NormalDriving::onExit() { kinematics->stop(); }