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

    // WIPE THE PID MEMORY! 
    // This stops the massive D-term kick that was slamming him into your hand.
    pid->reset();
}

void Mode_MaintainDistance::update(const RobotMood& currentMood) {
    float currentDistance = distSensor->getDistanceCM();
    
    // ==========================================
    // THE SOFTWARE CLUTCH (You missed this!)
    // ==========================================
    // If the distance suddenly jumps beyond our play zone (e.g., > 60cm), 
    // the Brain is currently running its 800ms verification timer.
    // Cut the motors instantly so he doesn't violently lurch while thinking!
    if (currentDistance > 60.0f) { // To-do: puth this 60 onto factory defaults and ConfigurationManager (NVS Storage) as "MAINTAIN_DISTANCE_EXIT_DISTANCE_CM"
        kinematics->stop();
        pid->reset(); // Keep memory clean while clutched
        return;       // Skip the PID math entirely
    }

    // Calculate dt (Delta Time) in seconds dynamically from the Master Clock!
    float dt = SystemConfig::MAIN_LOOP_TICK_RATE_MS / 1000.0f;
    
    float correction = pid->compute(Config.MAINTAIN_DISTANCE_CM, currentDistance, dt);
    float finalSpeed = correction * currentMood.speedMultiplier;
    kinematics->rawDrive(-finalSpeed, -finalSpeed);
}