#include "Mode_BrainDead.h"
#include "utils/RemoteLogger.h"

extern RemoteLogger logger;

Mode_BrainDead::Mode_BrainDead(KinematicsEngine* kinEngine) : kinematics(kinEngine) {}

void Mode_BrainDead::onEnter(const GlobalDataBank& robotData) {
    logger.println("[MODE] Entered BRAIN_DEAD. Actuators neutralized for data collection.");
    if (kinematics) kinematics->stop();
}

// FIX: Added 'const' and '&' here as well
void Mode_BrainDead::update(const RobotMood& currentMood, const GlobalDataBank& robotData) {
    // Actively force the kinematics engine to output 0 PWM.
    // This ensures no residual math commands the motors.
    if (kinematics) kinematics->stop(); 
}

void Mode_BrainDead::onExit() {
    logger.println("[MODE] Exiting BRAIN_DEAD.");
}