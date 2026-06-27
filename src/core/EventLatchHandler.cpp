#include "core/EventLatchHandler.h"
#include "config/ConfigurationManager.h"
#include <cmath> 

EventLatchHandler::EventLatchHandler() { 
    smoothedTotalEnergy = 0.0f;
    isHandling = false;
    lastDistance = -1.0f;
    lastAngles = {0, 0, 0, 0, false, 0};
}

SemanticEvents EventLatchHandler::processEvents(const GlobalDataBank& robotData) {
    SemanticEvents events; 

    // 1. CALCULATE DELTAS & ENERGY (Moved from gatherPerception)
    float currentDistance = (robotData.health.hardwareBitmask & Comms::HealthBit::SONAR_OK) ? robotData.sensors.distanceCM : -1.0f;
    float distanceDelta = (currentDistance != lastDistance && lastDistance > 0.0f) ? (currentDistance - lastDistance) : 0.0f;
    lastDistance = currentDistance;

    float rawYawEnergy = 0.0f, rawPitchEnergy = 0.0f, rawRollEnergy = 0.0f;
    
    if (robotData.health.hardwareBitmask & Comms::HealthBit::IMU_OK) {
        auto getShortestAngleDelta = [](float current, float previous) {
            float delta = current - previous;
            if (delta > 180.0f) delta -= 360.0f; else if (delta < -180.0f) delta += 360.0f;
            return delta;
        };
        rawYawEnergy = std::abs(getShortestAngleDelta(robotData.physics.imuAngles.yaw, lastAngles.yaw)) / 0.01f;
        rawPitchEnergy = std::abs(getShortestAngleDelta(robotData.physics.imuAngles.pitch, lastAngles.pitch)) / 0.01f;
        rawRollEnergy = std::abs(getShortestAngleDelta(robotData.physics.imuAngles.roll, lastAngles.roll)) / 0.01f;
        
        lastAngles.yaw = robotData.physics.imuAngles.yaw;
        lastAngles.pitch = robotData.physics.imuAngles.pitch;
        lastAngles.roll = robotData.physics.imuAngles.roll;
    }

    float totalRawEnergy = rawYawEnergy + rawPitchEnergy + rawRollEnergy;
    smoothedTotalEnergy = (SysConfig.ENERGY_EMA_ALPHA * totalRawEnergy) + (SysConfig.ENERGY_EMA_BETA * smoothedTotalEnergy);

    events.smoothedTotalEnergy = smoothedTotalEnergy; // Save the smoothed version and pass it top BehaviourEngine!

    // 2. DETERMINISTIC C++ PERCEPTION
    events.isFreeFalling = (robotData.physics.imuAngles.gForce < 0.25f);

    if (!robotData.actuators.isDriving && totalRawEnergy > SysConfig.STEADY_HOLD_ENERGY_MAX) {
        isHandling = true; 
    } else if (smoothedTotalEnergy < SysConfig.PERFECTLY_STILL_ENERGY) {
        isHandling = false; 
    }
    events.isHandling = isHandling;
    events.isAbsolutelyStill = (!robotData.actuators.isDriving && !events.isHandling && smoothedTotalEnergy < SysConfig.PERFECTLY_STILL_ENERGY);

    // 3. STATIC ORIENTATIONS
    float pitch = robotData.physics.imuAngles.pitch;
    float roll = robotData.physics.imuAngles.roll;

    // CLEAR THE DEFAULT STRUCT LATCH FIRST
    events.isUpright = false; 

    // ONLY ONE WILL TRIGGER (Mutual Exclusivity)
    if (std::abs(roll) > 135.0f || std::abs(pitch) > 135.0f) events.isUpsideDown = true;
    else if (pitch > 70.0f) events.isNoseUp = true;
    else if (pitch < -70.0f) events.isNoseDown = true;
    else if (roll > 70.0f) events.isTippedRight = true;
    else if (roll < -70.0f) events.isTippedLeft = true;
    else if (std::abs(pitch) < 30.0f && std::abs(roll) < 30.0f) events.isUpright = true;

    // Note: If the robot is at a weird 45-degree diagonal, all orientation flags will safely be false until it settles.

    // ==========================================================
    // NEURAL NETWORK LAYER HOOK
    // ==========================================================
    
    // The following flags are left as 'false' here. 
    // You will overwrite them with the output of your TensorFlow Lite model 
    // in the main loop AFTER this function returns.
    
    // events.isStuck = runAIPerceptionModel(p).isStuck;
    // events.hazardDetected = runAIPerceptionModel(p).hazardDetected;
    // events.isBeingTeased = runAIPerceptionModel(p).isBeingTeased;
    // events.isBeingPushed = runAIPerceptionModel(p).isBeingPushed;

    return events;
}