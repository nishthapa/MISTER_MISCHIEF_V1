#include "behaviours/Mode_CompassLock.h"
#include "utils/RemoteLogger.h" // <-- ADD THIS INCLUDE
#include <Arduino.h>

Mode_CompassLock::Mode_CompassLock(I_IMU* i, XY160D_MotorDriver* m, PIDController* p) {
    imu = i; motors = m; pid = p;
    targetYaw = 0.0f;
}

void Mode_CompassLock::onEnter() {
    // 1. Take the Snapshot! 
    // The exact moment you pick him up, remember where he is looking.
    targetYaw = imu->getAngles().yaw;
    logger.printf("Compass Lock engaged! Locking heading to: %.1f degrees\n", targetYaw);
}

void Mode_CompassLock::update(const RobotMood& currentMood) {
    float currentYaw = imu->getAngles().yaw;
    
    // 2. Calculate the error
    // Note: To make this robust across the 180/-180 boundary, you will want a shortest-path calculation here
    float error = targetYaw - currentYaw;
    if (error > 180.0f) error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    // 3. Do the Math
    // We feed the error directly into the compute function (assuming setpoint is 0 relative to the error)
    float correction = pid->compute(0.0f, -error, 0.01f); 

    // 4. APPLY THE MOOD 
    // If he is angry, he snaps to the heading much more violently
    float finalCorrection = correction * currentMood.pidAggression;

    // 5. Move Muscles (Pivot in place)
    // To rotate in place, one track goes forward, the other goes backward
    motors->drive(finalCorrection, -finalCorrection);
}

void Mode_CompassLock::onExit() {
    logger.println("Compass Lock disengaged. Returning to normal.");
    //logger.println("Compass Lock disengaged. Returning to normal.");
    motors->stop();
}