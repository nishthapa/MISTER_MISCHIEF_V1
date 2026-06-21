#include <Arduino.h>
#include "core/BehaviourEngine.h"
#include "behaviours/Mode_ObstacleAvoidance.h"
#include "behaviours/Mode_NormalDriving.h"
#include "behaviours/Mode_CompassLock.h"
#include "behaviours/Mode_MaintainDistance.h"
#include "behaviours/Mode_Dizzy.h"
#include "behaviours/Mode_DeepSleep.h"
#include "behaviours/Mode_Teleop.h" // <--- NEW TELEOP MODE
#include "behaviours/Mode_Diagnostics.h"
#include "behaviours/Mode_Autotune.h"
#include "config/ConfigurationManager.h" 

BehaviourEngine::BehaviourEngine(Mode_ObstacleAvoidance* obs, Mode_NormalDriving* norm, 
                                 Mode_CompassLock* comp, Mode_MaintainDistance* dist, 
                                 Mode_Dizzy* diz, Mode_DeepSleep* sleep, Mode_Teleop* teleop,
                                 Mode_Diagnostics* diag, Mode_AutoTune* autot) {
    obstacleMode = obs; normalMode = norm; compassMode = comp;
    distanceMode = dist; dizzyMode = diz; sleepMode = sleep; teleopMode = teleop;
    diagnosticMode = diag; autotuneMode = autot;

    activeMode = normalMode; previousMode = nullptr; activeMood = Moods::HAPPY;
    isGroggyPhase = false; lastDistance = -1.0f; lastAngles = {0,0,0,0,false,0};
}

void BehaviourEngine::init(bool isColdBoot) {
    if (isColdBoot) { activeMood = Moods::GROGGY; isGroggyPhase = true; coldBootTime = millis(); } 
    else { activeMood = Moods::HAPPY; isGroggyPhase = false; }
    
    latchHandler.reset();
    
    // SAFELY PROJECT INITIAL STATE TO THE BUS
    portENTER_CRITICAL(&globalDataBusLock);
    CurrentRobotData.cognition.systemMode = mapModeToEnum(activeMode);
    CurrentRobotData.cognition.robotMood = activeMood.id;
    portEXIT_CRITICAL(&globalDataBusLock);
    
    // FIX 1: Pass the global state to onEnter during boot!
    activeMode->onEnter(CurrentRobotData); 
    previousMode = activeMode;
}

void BehaviourEngine::changeMode(IRobotMode* newMode) { 
    if (newMode != nullptr && newMode != activeMode) {
        if (activeMode != nullptr) activeMode->onExit(); // Safely shut down the old mode
        previousMode = activeMode;
        activeMode = newMode;
        
        // Boot up the new mode immediately!
        activeMode->onEnter(CurrentRobotData); 
        
        // SAFELY PROJECT NEW COMMANDED STATE TO THE BUS
        portENTER_CRITICAL(&globalDataBusLock);
        CurrentRobotData.cognition.systemMode = mapModeToEnum(activeMode);
        CurrentRobotData.cognition.robotMood = activeMood.id;
        portEXIT_CRITICAL(&globalDataBusLock);
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

    // 1. Check the Command Bus for System Overrides
    bool isDiagnosticRequested = false;
    bool isAutotuneRequested = false;

    portENTER_CRITICAL(&hardwareCmdLock);
    isDiagnosticRequested = HardwareCommands.requestMotorTest;
    isAutotuneRequested = HardwareCommands.requestAutotune;
    portEXIT_CRITICAL(&hardwareCmdLock);

    // 2. Highest Priority: Developer Overrides
    if (isDiagnosticRequested) return diagnosticMode;
    if (isAutotuneRequested) return autotuneMode;
    
    // Priority 3: Survival Sequences (Cannot be interrupted easily)
    if (activeMode == obstacleMode && !obstacleMode->isSequenceComplete()) return obstacleMode; 
    
    if (activeMode == dizzyMode) {
        if (events.dizzyFinished) return normalMode;
        return dizzyMode;
    }

    // Priority 4: Handling Interrupts
    if (events.dizzyTriggered) return dizzyMode;
    
    if (activeMode == compassMode) {
        if (events.safelyLanded) return normalMode;
        return compassMode;
    } else if (events.readyForCompassLock) return compassMode;

    // Priority 5: The Games
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

    // Enforce the 500ms Watchdog Timer so that autonomous brain is resumed
    // if the connection is dropped or the app stops sending control commands
    TeleopCommands.checkFailsafe();
    teleopSnapshot = TeleopCommands;
    portEXIT_CRITICAL(&teleopCmdLock);

    // If the phone is connected via BLE, we immediately force the manual mode.
    // This ignores whatever Config.BRAIN_ACTIVE is set to.
    // If an authorized client explicitly holds the token, force manual mode.
    //if (teleopSnapshot.isConnected) {
    if (teleopSnapshot.isOverrideActive) { // 🚨 CHANGED: isConnected -> isOverrideActive
        if (activeMode != teleopMode) { // Changed from GLOBAL_MODE check
            if (activeMode != nullptr) activeMode->onExit();
            previousMode = activeMode;
            activeMode = teleopMode;
            activeMode->onEnter(robotData);
        }
        
        activeMode->update(activeMood, robotData);

        // SAFELY PROJECT OVERRIDE STATE TO THE BUS
        portENTER_CRITICAL(&globalDataBusLock);
        CurrentRobotData.cognition.systemMode = SystemMode::MANUAL_OVERRIDE;
        CurrentRobotData.cognition.robotMood = activeMood.id;
        portEXIT_CRITICAL(&globalDataBusLock);

        return;
    }

    if (!SysConfig.BRAIN_ACTIVE) {
        // SAFELY PROJECT OVERRIDE STATE TO THE BUS
        portENTER_CRITICAL(&globalDataBusLock);
        CurrentRobotData.cognition.systemMode = SystemMode::MANUAL_OVERRIDE;
        CurrentRobotData.cognition.robotMood = activeMood.id;
        portEXIT_CRITICAL(&globalDataBusLock);

        // FIX 2: Pass sensorState to the manual override update!
        if (activeMode) activeMode->update(activeMood, robotData);
        return;
    }

    PerceptionData perception = gatherPerception(robotData);

    // Get the current enum locally
    SystemMode currentSysMode = mapModeToEnum(activeMode);
    
    // Process physical data into semantic meaning
    SemanticEvents recentEvents = latchHandler.processEvents(perception, currentSysMode);

    // Determine the next mode BEFORE taking the lock
    IRobotMode* nextMode = determineNextMode(recentEvents);

    // Handle the mode transition locally
    if (nextMode != activeMode) {
        if (activeMode != nullptr) activeMode->onExit();
        previousMode = activeMode;
        activeMode = nextMode;
        if (activeMode != nullptr) activeMode->onEnter(robotData); 
    }

    // NOW, TAKE THE LOCK and write the final, agreed-upon reality
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

    // Map the Human Interaction triggers / latches using the getters
    CurrentRobotData.events.isHandTeasing = latchHandler.getIsHandTeasing();
    CurrentRobotData.events.isHandVanishing = latchHandler.getIsHandVanishing();
    CurrentRobotData.events.isHandling = latchHandler.getIsHandling();
    CurrentRobotData.events.hasExperiencedLift = latchHandler.getHasExperiencedLift();
    CurrentRobotData.events.isLowering = latchHandler.getIsLowering();
    CurrentRobotData.events.hasLanded = latchHandler.getHasLanded();
    CurrentRobotData.events.isDizzy = latchHandler.getIsDizzy();

    // Map the Cognitive State
    //CurrentRobotData.cognition.systemMode = currentSysMode;

    // Map the Cognitive State using the NEW mode
    CurrentRobotData.cognition.systemMode = mapModeToEnum(activeMode);
    
    //CurrentRobotData.cognition.robotMood = activeMood;
    // Pass ONLY the 1-byte ID to the telemetry bus, not the whole mood struct!
    CurrentRobotData.cognition.robotMood = activeMood.id;
    
    // RELEASE THE LOCK
    portEXIT_CRITICAL(&globalDataBusLock);

    // Proceed to mode switching...
    //IRobotMode* nextMode = determineNextMode(recentEvents);

    // ... Determine the next mode ...
    // if (nextMode != activeMode) {
    //     if (activeMode != nullptr) activeMode->onExit();
    //     previousMode = activeMode; // Mode context switch
    //     activeMode = nextMode;
    //     if (activeMode != nullptr) activeMode->onEnter(robotData); // <--- PASS IT HERE
    //     // GLOBAL_MODE = mapModeToEnum(activeMode); // GLOBAL_MODE now directly resides in GlobalDataBus
    // }

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