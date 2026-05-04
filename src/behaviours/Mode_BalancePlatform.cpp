#include "behaviours/Mode_BalancePlatform.h"
#include <Arduino.h>

Mode_BalancePlatform::Mode_BalancePlatform(MPU6050_IMU* i, XY160D_MotorDriver* m, PIDController* p) {
    imu = i;
    motors = m;
    pid = p;
}

void Mode_BalancePlatform::onEnter() {
    Serial.println("MODE: Balance Platform initialized. Engage motors.");
}

void Mode_BalancePlatform::update(const RobotMood& currentMood) {
    // 1. Read Senses
    FusedAngles angles = imu->getAngles();
    
    // 2. Do Math (Target is 0.0 degrees, perfectly flat)
    float baseCorrection = pid->compute(0.0f, angles.pitch, 0.01f); // Assuming 100Hz loop (dt = 0.01)

    // 3. APPLY THE MOOD 
    // If the robot is ANGRY, it fights the balance harder and twitchier.
    float finalCorrection = baseCorrection * currentMood.pidAggression;

    // 4. Move Muscles (Balance platform requires both tracks to move together)
    motors->drive(finalCorrection, finalCorrection);
}

void Mode_BalancePlatform::onExit() {
    Serial.println("MODE: Balance Platform terminated. Cutting power.");
    motors->stop();
}