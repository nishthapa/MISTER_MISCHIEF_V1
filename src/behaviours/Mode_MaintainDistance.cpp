#include "behaviours/Mode_MaintainDistance.h"
#include "utils/RemoteLogger.h"
// #include "config/PersonalityConfig.h" // Safe to DELETE since we're now pulling these values from the NVS-backed ConfigurationManager
#include "config/SystemConfig.h"      // <-- For the master clock
#include "config/ConfigurationManager.h"

Mode_MaintainDistance::Mode_MaintainDistance(KinematicsEngine* k, PIDController* p) {
    kinematics = k; pid = p;
}

void Mode_MaintainDistance::onEnter(const volatile GlobalSensorState& sensorState) {
    logger.println("Mister Mischief is maintaining distance!");

    // WIPE THE PID MEMORY! 
    // This stops the massive D-term kick that was slamming him into your hand.
    pid->reset();
}

void Mode_MaintainDistance::update(const RobotMood& currentMood, const volatile GlobalSensorState& sensorState) {
    float currentDistance = sensorState.distanceCM; // READ FROM GLOBALSENSORSTATE MEMORY!
    
    // THE SOFTWARE CLUTCH
    // If the book is actually removed (jumps > 60cm), pause the motors
    if (currentDistance > 60.0f) { 
        kinematics->stop();
        pid->reset(); 
        return;       
    }

    // ==========================================
    // THE ALIASING FIX (Decouple PID from Brain)
    // ==========================================
    static float lastProcessedDistance = -1.0f;
    static float lastCorrection = 0.0f;

    // ONLY run the PID math when a fresh physical ping clears the 40ms cache!
    if (currentDistance != lastProcessedDistance) {
        
        // The true physical time elapsed between sensor updates is ~40ms (0.04s)
        float trueDt = 0.04f; 
        
        lastCorrection = pid->compute(Config.MAINTAIN_DISTANCE_CM, currentDistance, trueDt);
        lastProcessedDistance = currentDistance;
    }

    // Apply the perfectly smoothed correction to the tracks continuously
    float finalSpeed = lastCorrection * currentMood.speedMultiplier;
    kinematics->rawDrive(-finalSpeed, -finalSpeed);
}