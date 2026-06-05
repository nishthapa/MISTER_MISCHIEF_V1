#include "behaviours/Mode_CompassLock.h"
#include "utils/RemoteLogger.h" // <-- ADD THIS INCLUDE

Mode_CompassLock::Mode_CompassLock(KinematicsEngine* k) {
    kinematics = k;
    targetYaw = 0.0f;
}

void Mode_CompassLock::onEnter(const volatile GlobalSensorState& sensorState) {
    // 1. Take the Snapshot! 
    // The exact moment you pick him up, remember where he is looking.
    targetYaw = sensorState.imuAngles.yaw; // Read from Memory!
    logger.printf("Compass Lock engaged! Locking heading to: %.1f degrees\n", targetYaw);
}

void Mode_CompassLock::update(const RobotMood& currentMood, const volatile GlobalSensorState& sensorState) {
    // Engine automatically routes to Point Turn profile because base speed = 0
    kinematics->navigateToHeading(targetYaw, sensorState.imuAngles.yaw, 0.0f, currentMood.pidAggression);
}

void Mode_CompassLock::onExit() { kinematics->stop(); }