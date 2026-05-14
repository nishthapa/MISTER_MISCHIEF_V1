#include "behaviours/Mode_Dizzy.h"
#include <Arduino.h>
#include "utils/RemoteLogger.h"

Mode_Dizzy::Mode_Dizzy(XY160D_MotorDriver* m) {
    motors = m;
}

void Mode_Dizzy::onEnter() {
    logger.println("Mister Mischief is dizzy!");
}

void Mode_Dizzy::update(const RobotMood& currentMood) {
    // Use the ESP32's internal clock to generate a sine wave for the motors
    // This creates a wobbly, staggering drive path
    float timeSec = millis() / 1000.0f;
    
    // Left motor pulses differently than the right motor
    int leftWobble = sin(timeSec * 3.0f) * 60.0f; 
    int rightWobble = cos(timeSec * 2.5f) * 60.0f; 

    motors->drive(leftWobble, rightWobble);
}