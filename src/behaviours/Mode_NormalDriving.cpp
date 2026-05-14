#include "behaviours/Mode_NormalDriving.h"
#include "config/PersonalityConfig.h" // For the default cruising speed constant
#include "utils/RemoteLogger.h" // For logging when we enter normal driving mode
#include <Arduino.h>

Mode_NormalDriving::Mode_NormalDriving(XY160D_MotorDriver* m) {
    motors = m;
}

void Mode_NormalDriving::onEnter() {
    logger.println("Mister Mischief is cruising in Normal Driving Mode...");
}

void Mode_NormalDriving::update(const RobotMood& currentMood) {
    // Pull the speed from the new config file!
    float finalSpeed = PersonalityConfig::NORMAL_CRUISING_SPEED * currentMood.speedMultiplier;
    
    motors->drive(finalSpeed, finalSpeed);
}

void Mode_NormalDriving::onExit() {
    motors->stop();
}