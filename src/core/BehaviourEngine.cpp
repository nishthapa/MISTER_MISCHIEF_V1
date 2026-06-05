#include "core/BehaviourEngine.h"
#include <Arduino.h>
#include "behaviours/Mode_ObstacleAvoidance.h"
#include "behaviours/Mode_NormalDriving.h"
#include "behaviours/Mode_CompassLock.h"
#include "behaviours/Mode_MaintainDistance.h"
#include "behaviours/Mode_Dizzy.h"
#include "behaviours/Mode_DeepSleep.h"
#include "config/ConfigurationManager.h" 

BehaviourEngine::BehaviourEngine(I_IMU* i, I_DistanceSensor* s, 
                                 Mode_ObstacleAvoidance* obs, Mode_NormalDriving* norm, 
                                 Mode_CompassLock* comp, Mode_MaintainDistance* dist, 
                                 Mode_Dizzy* diz, Mode_DeepSleep* sleep) {
    imu = i; sonar = s;
    obstacleMode = obs; normalMode = norm; compassMode = comp;
    distanceMode = dist; dizzyMode = diz; sleepMode = sleep;

    activeMode = normalMode; previousMode = nullptr; activeMood = Moods::HAPPY;
    isGroggyPhase = false; lastDistance = -1.0f; lastAngles = {0,0,0,0,false,0};
}

void BehaviourEngine::init(bool isColdBoot) {
    if (isColdBoot) { activeMood = Moods::GROGGY; isGroggyPhase = true; coldBootTime = millis(); } 
    else { activeMood = Moods::HAPPY; isGroggyPhase = false; }
    latchHandler.reset();
    GLOBAL_MODE = mapModeToEnum(activeMode);
    activeMode->onEnter();
    previousMode = activeMode;
}

void BehaviourEngine::changeMode(IRobotMode* newMode) { if (newMode != nullptr) activeMode = newMode; }
const char* BehaviourEngine::getActiveModeName() const { return activeMode->getName(); }
const char* BehaviourEngine::getActiveMoodName() const { return "HAPPY"; } // Simplified for brevity

SystemMode BehaviourEngine::mapModeToEnum(IRobotMode* mode) {
    if (mode == normalMode) return SystemMode::MODE_NORMAL_DRIVING;
    if (mode == obstacleMode) return SystemMode::MODE_OBSTACLE_AVOIDANCE;
    if (mode == distanceMode) return SystemMode::MODE_MAINTAIN_DISTANCE;
    if (mode == compassMode) return SystemMode::MODE_COMPASS_LOCK;
    if (mode == dizzyMode) return SystemMode::MODE_DIZZY;
    if (mode == sleepMode) return SystemMode::MODE_DEEP_SLEEP;
    return SystemMode::MODE_NORMAL_DRIVING;
}

PerceptionData BehaviourEngine::gatherPerception(const volatile GlobalSensorState& sensorState) {
    PerceptionData p;
    
    // NO MORE HARDWARE CALLS! Just read the passed-in state!
    p.currentDistance = sensorState.distanceCM;
    p.currentGForce = sensorState.imuAngles.gForce;

    p.distanceDelta = (p.currentDistance != lastDistance && lastDistance > 0.0f) ? (p.currentDistance - lastDistance) : 0.0f;

    auto getShortestAngleDelta = [](float current, float previous) {
        float delta = current - previous;
        if (delta > 180.0f) delta -= 360.0f;
        else if (delta < -180.0f) delta += 360.0f;
        return delta;
    };
    
    p.rawYawEnergy = abs(getShortestAngleDelta(sensorState.imuAngles.yaw, lastAngles.yaw)) / 0.01f;
    p.rawPitchEnergy = abs(getShortestAngleDelta(sensorState.imuAngles.pitch, lastAngles.pitch)) / 0.01f;
    p.rawRollEnergy = abs(getShortestAngleDelta(sensorState.imuAngles.roll, lastAngles.roll)) / 0.01f;
    p.totalRawEnergy = p.rawYawEnergy + p.rawPitchEnergy + p.rawRollEnergy;
    p.isUpright = (abs(sensorState.imuAngles.pitch) < Config.UPRIGHT_ANGLE_TOLERANCE && abs(sensorState.imuAngles.roll) < Config.UPRIGHT_ANGLE_TOLERANCE);

    // Deep copy the volatile struct angles into our local history
    lastAngles.yaw = sensorState.imuAngles.yaw;
    lastAngles.pitch = sensorState.imuAngles.pitch;
    lastAngles.roll = sensorState.imuAngles.roll;
    lastAngles.gForce = sensorState.imuAngles.gForce;
    
    lastDistance = p.currentDistance;
    return p;
}

// === THE PURE DECISION SWITCHBOARD ===
IRobotMode* BehaviourEngine::determineNextMode(const SemanticEvents& events) {
    
    // Priority 1: Survival Sequences (Cannot be interrupted easily)
    if (activeMode == obstacleMode && !obstacleMode->isSequenceComplete()) return obstacleMode; 
    
    if (activeMode == dizzyMode) {
        if (events.dizzyFinished) return normalMode;
        return dizzyMode;
    }

    // Priority 2: Handling Interrupts
    if (events.dizzyTriggered) return dizzyMode;
    
    if (activeMode == compassMode) {
        if (events.safelyLanded) return normalMode;
        return compassMode;
    } else if (events.readyForCompassLock) return compassMode;

    // Priority 3: The Games
    if (activeMode == distanceMode) {
        if (events.frustrationPeaked) return dizzyMode;
        if (events.targetVanished) return normalMode;
        return distanceMode;
    }

    if (events.hazardDetected) {
        if (events.teaseConfirmed) return distanceMode;
        return obstacleMode;
    }

    return normalMode;
}

void BehaviourEngine::update(const volatile GlobalSensorState& sensorState) {
    if (!Config.BRAIN_ACTIVE) {
        GLOBAL_MODE = SystemMode::MANUAL_OVERRIDE;
        if (activeMode) activeMode->update(activeMood);
        return;
    }

    PerceptionData perception = gatherPerception(sensorState);
    
    // Pass the physics to the tracker to get clean boolean events!
    SemanticEvents events = latchHandler.processEvents(perception, GLOBAL_MODE);

    IRobotMode* nextMode = determineNextMode(events);

    if (nextMode != activeMode) {
        if (activeMode != nullptr) activeMode->onExit();
        previousMode = activeMode;
        activeMode = nextMode;
        if (activeMode != nullptr) activeMode->onEnter();
        GLOBAL_MODE = mapModeToEnum(activeMode); 
    }

    if (activeMode == normalMode || activeMode == obstacleMode) imu->setFilterBeta(0.01f);
    else imu->setFilterBeta(Config.MADGWICK_FILTER_BETA); 

    if (activeMode != nullptr) activeMode->update(activeMood);
}