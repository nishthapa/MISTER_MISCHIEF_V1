#include "core/BehaviourEngine.h"
#include <Arduino.h>
#include "behaviours/Mode_ObstacleAvoidance.h"
#include "behaviours/Mode_NormalDriving.h"
#include "behaviours/Mode_CompassLock.h"
#include "behaviours/Mode_MaintainDistance.h"
#include "behaviours/Mode_Dizzy.h"
#include "behaviours/Mode_DeepSleep.h"
#include "behaviours/Mode_Teleop.h" // <--- NEW TELEOP MODE
#include "config/ConfigurationManager.h" 

BehaviourEngine::BehaviourEngine(Mode_ObstacleAvoidance* obs, Mode_NormalDriving* norm, 
                                 Mode_CompassLock* comp, Mode_MaintainDistance* dist, 
                                 Mode_Dizzy* diz, Mode_DeepSleep* sleep, Mode_Teleop* teleop) {
    obstacleMode = obs; normalMode = norm; compassMode = comp;
    distanceMode = dist; dizzyMode = diz; sleepMode = sleep; teleopMode = teleop;

    activeMode = normalMode; previousMode = nullptr; activeMood = Moods::HAPPY;
    isGroggyPhase = false; lastDistance = -1.0f; lastAngles = {0,0,0,0,false,0};
}

void BehaviourEngine::init(bool isColdBoot) {
    if (isColdBoot) { activeMood = Moods::GROGGY; isGroggyPhase = true; coldBootTime = millis(); } 
    else { activeMood = Moods::HAPPY; isGroggyPhase = false; }
    
    latchHandler.reset();
    GLOBAL_MODE = mapModeToEnum(activeMode);
    
    // FIX 1: Pass the global state to onEnter during boot!
    activeMode->onEnter(CurrentRobotData); 
    previousMode = activeMode;
}

// void BehaviourEngine::changeMode(IRobotMode* newMode) { if (newMode != nullptr) activeMode = newMode; }

void BehaviourEngine::changeMode(IRobotMode* newMode) { 
    if (newMode != nullptr && newMode != activeMode) {
        if (activeMode != nullptr) activeMode->onExit(); // Safely shut down the old mode
        previousMode = activeMode;
        activeMode = newMode;
        
        // Boot up the new mode immediately!
        activeMode->onEnter(CurrentRobotData); 
        
        GLOBAL_MODE = mapModeToEnum(activeMode); 
    } 
}

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

PerceptionData BehaviourEngine::gatherPerception(const GlobalDataBank& robotData) {
    PerceptionData p;

    // --- HARDWARE SAFETY CHECKS (Bitmask Verification) ---
    
    // 1. Sonar Safety Check
    if (robotData.health.hardwareBitmask & Comms::HealthBit::SONAR_OK) {
        p.currentDistance = robotData.sensors.distanceCM;
    } else {
        // Safe fallback if Sonar is dead or disconnected
        p.currentDistance = -1.0f; 
    }
    

    p.currentGForce = robotData.physics.imuAngles.gForce;

    p.distanceDelta = (p.currentDistance != lastDistance && lastDistance > 0.0f) ? (p.currentDistance - lastDistance) : 0.0f;

    auto getShortestAngleDelta = [](float current, float previous) {
        float delta = current - previous;
        if (delta > 180.0f) delta -= 360.0f;
        else if (delta < -180.0f) delta += 360.0f;
        return delta;
    };
    
    // 2. IMU Safety Check (Wraps ALL kinematic math)
    if (robotData.health.hardwareBitmask & Comms::HealthBit::IMU_OK) {
        p.rawYawEnergy = abs(getShortestAngleDelta(robotData.physics.imuAngles.yaw, lastAngles.yaw)) / 0.01f;
        p.rawPitchEnergy = abs(getShortestAngleDelta(robotData.physics.imuAngles.pitch, lastAngles.pitch)) / 0.01f;
        p.rawRollEnergy = abs(getShortestAngleDelta(robotData.physics.imuAngles.roll, lastAngles.roll)) / 0.01f;
        p.totalRawEnergy = p.rawYawEnergy + p.rawPitchEnergy + p.rawRollEnergy;
        p.isUpright = (abs(robotData.physics.imuAngles.pitch) < SysConfig.UPRIGHT_ANGLE_TOLERANCE && abs(robotData.physics.imuAngles.roll) < SysConfig.UPRIGHT_ANGLE_TOLERANCE);

        // Deep copy the volatile struct angles into our local history
        lastAngles.yaw = robotData.physics.imuAngles.yaw;
        lastAngles.pitch = robotData.physics.imuAngles.pitch;
        lastAngles.roll = robotData.physics.imuAngles.roll;
        lastAngles.gForce = robotData.physics.imuAngles.gForce;
    } else {
        // Safe fallbacks if IMU is dead or disconnected
        p.currentGForce = 1.0f; // 1G is normal resting gravity
        p.currentYaw = 0.0f;
        p.currentPitch = 0.0f;
        p.currentRoll = 0.0f;

        p.rawYawEnergy = 0.0f;
        p.rawPitchEnergy = 0.0f;
        p.rawRollEnergy = 0.0f;
        p.totalRawEnergy = 0.0f;

        p.isUpright = true; // Assume upright if we can't measure
    }
    
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

void BehaviourEngine::update(const GlobalDataBank& robotData) {

    // THE SAFE DYNAMIC OVERRIDE:
    // 1. Take a safe snapshot of the teleop bus
    TeleopCommandBus teleopSnapshot;
    portENTER_CRITICAL(&teleopCmdLock);
    teleopSnapshot = TeleopCommands;
    portEXIT_CRITICAL(&teleopCmdLock);

    // If the phone is connected via BLE, we immediately force the manual mode.
    // This ignores whatever Config.BRAIN_ACTIVE is set to.
    if (teleopSnapshot.isConnected) {
        if (GLOBAL_MODE != SystemMode::MANUAL_OVERRIDE) {
            if (activeMode != nullptr) activeMode->onExit();
            activeMode = teleopMode;
            activeMode->onEnter(robotData);
            GLOBAL_MODE = SystemMode::MANUAL_OVERRIDE;
        }
        
        // Update the teleop mode with the physics
        activeMode->update(activeMood, robotData);
        return;
    }

    if (!SysConfig.BRAIN_ACTIVE) {
        GLOBAL_MODE = SystemMode::MANUAL_OVERRIDE;
        // FIX 2: Pass sensorState to the manual override update!
        if (activeMode) activeMode->update(activeMood, robotData);
        return;
    }

    PerceptionData perception = gatherPerception(robotData);

    // Map the internal metrics for Foxglove Tuning
    // CurrentRobotData.perception.distanceDelta = perception.distanceDelta;
    // CurrentRobotData.perception.totalRawEnergy = perception.totalRawEnergy;
    // CurrentRobotData.perception.rawYawEnergy = perception.rawYawEnergy;
    // CurrentRobotData.perception.rawPitchEnergy = perception.rawPitchEnergy;
    // CurrentRobotData.perception.rawRollEnergy = perception.rawRollEnergy;
    // CurrentRobotData.perception.currentGForce = perception.currentGForce;
    
    // Process physical data into semantic meaning
    SemanticEvents recentEvents = latchHandler.processEvents(perception, GLOBAL_MODE);

    // TAKE THE LOCK
    portENTER_CRITICAL(&globalDataBusLock);

    // 1. Map the internal metrics for live telemetry Tuning (MOVED INSIDE THE LOCK)
    CurrentRobotData.perception.distanceDelta = perception.distanceDelta;
    CurrentRobotData.perception.totalRawEnergy = perception.totalRawEnergy;
    CurrentRobotData.perception.rawYawEnergy = perception.rawYawEnergy;
    CurrentRobotData.perception.rawPitchEnergy = perception.rawPitchEnergy;
    CurrentRobotData.perception.rawRollEnergy = perception.rawRollEnergy;
    CurrentRobotData.perception.currentGForce = perception.currentGForce;
    
    // Map the Triggers
    CurrentRobotData.events.hazardDetected = recentEvents.hazardDetected;
    CurrentRobotData.events.teaseConfirmed = recentEvents.teaseConfirmed;
    CurrentRobotData.events.targetVanished = recentEvents.targetVanished;
    CurrentRobotData.events.dizzyTriggered = recentEvents.dizzyTriggered;
    CurrentRobotData.events.dizzyFinished = recentEvents.dizzyFinished;
    CurrentRobotData.events.readyForCompassLock = recentEvents.readyForCompassLock;
    CurrentRobotData.events.safelyLanded = recentEvents.safelyLanded;
    CurrentRobotData.events.frustrationPeaked = recentEvents.frustrationPeaked;

    // Map the Internal Metrics using the getters
    CurrentRobotData.events.dizzyBarYaw = latchHandler.getDizzyBarYaw();
    CurrentRobotData.events.dizzyBarPitch = latchHandler.getDizzyBarPitch();
    CurrentRobotData.events.dizzyBarRoll = latchHandler.getDizzyBarRoll();
    CurrentRobotData.events.smoothedTotalEnergy = latchHandler.getSmoothedTotalEnergy();
    CurrentRobotData.events.frustrationLevel = latchHandler.getFrustrationLevel();

    CurrentRobotData.events.isHandTeasing = latchHandler.getIsHandTeasing();
    CurrentRobotData.events.isHandVanishing = latchHandler.getIsHandVanishing();
    CurrentRobotData.events.isHandling = latchHandler.getIsHandling();
    CurrentRobotData.events.hasExperiencedLift = latchHandler.getHasExperiencedLift();
    CurrentRobotData.events.isLowering = latchHandler.getIsLowering();
    CurrentRobotData.events.hasLanded = latchHandler.getHasLanded();
    CurrentRobotData.events.isDizzy = latchHandler.getIsDizzy();
    
    // RELEASE THE LOCK
    portEXIT_CRITICAL(&globalDataBusLock);

    // Proceed to mode switching...

    IRobotMode* nextMode = determineNextMode(recentEvents);

    // ... Determine next mode ...

    if (nextMode != activeMode) {
        if (activeMode != nullptr) activeMode->onExit();
        previousMode = activeMode;
        activeMode = nextMode;
        if (activeMode != nullptr) activeMode->onEnter(robotData); // <--- PASS IT HERE
        GLOBAL_MODE = mapModeToEnum(activeMode); 
    }

    // FIX 3: Use the Command Bus to change the IMU filter, safely locked!
    portENTER_CRITICAL(&hardwareCmdLock);
    if (activeMode == normalMode || activeMode == obstacleMode) {
        HardwareCommands.targetFilterBeta = 0.01f;
    } else {
        HardwareCommands.targetFilterBeta = SysConfig.MADGWICK_FILTER_BETA;
    }
    portEXIT_CRITICAL(&hardwareCmdLock);

    if (activeMode != nullptr) activeMode->update(activeMood, robotData);
}