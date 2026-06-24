#include <Arduino.h>
#include "core/BehaviourEngine.h"
#include "behaviours/Mode_ObstacleAvoidance.h"
#include "behaviours/Mode_NormalDriving.h"
#include "behaviours/Mode_CompassLock.h"
#include "behaviours/Mode_MaintainDistance.h"
#include "behaviours/Mode_Dizzy.h"
#include "behaviours/Mode_DeepSleep.h"
#include "behaviours/Mode_Teleop.h" 
#include "behaviours/Mode_Diagnostics.h"
#include "behaviours/Mode_Autotune.h"
#include "behaviours/Mode_BrainDead.h"
#include "config/ConfigurationManager.h"

BehaviourEngine::BehaviourEngine(IBehaviourDecider* decisionEngine,
                                Mode_ObstacleAvoidance* obs, Mode_NormalDriving* norm, 
                                Mode_CompassLock* comp, Mode_MaintainDistance* dist, 
                                Mode_Dizzy* diz, Mode_DeepSleep* sleep, Mode_Teleop* teleop,
                                Mode_Diagnostics* diag, Mode_AutoTune* autot,
                                Mode_BrainDead* deadMode) {
    decider = decisionEngine;
    obstacleMode = obs; normalMode = norm; compassMode = comp;
    distanceMode = dist; dizzyMode = diz; sleepMode = sleep; teleopMode = teleop;
    diagnosticMode = diag; autotuneMode = autot; brainDeadMode = deadMode;

    activeMode = normalMode; previousMode = nullptr; activeMood = Moods::HAPPY;
    isGroggyPhase = false; 
}

void BehaviourEngine::init(bool isColdBoot) {
    if (isColdBoot) { 
        activeMood = Moods::GROGGY; 
        isGroggyPhase = true; 
        coldBootTime = millis(); 
    } 
    else { 
        activeMood = Moods::HAPPY; 
        isGroggyPhase = false; 
    }
    
    portENTER_CRITICAL(&globalDataBusLock);
    CurrentRobotData.cognition.systemMode = mapModeToEnum(activeMode);
    CurrentRobotData.cognition.robotMood = activeMood.id;
    portEXIT_CRITICAL(&globalDataBusLock);
    
    activeMode->onEnter(CurrentRobotData);
    previousMode = activeMode;
}

void BehaviourEngine::changeMode(IRobotMode* newMode) { 
    if (newMode != nullptr && newMode != activeMode) {
        if (activeMode != nullptr) activeMode->onExit();
        previousMode = activeMode;
        activeMode = newMode;
        activeMode->onEnter(CurrentRobotData); 
        
        portENTER_CRITICAL(&globalDataBusLock);
        CurrentRobotData.cognition.systemMode = mapModeToEnum(activeMode);
        CurrentRobotData.cognition.robotMood = activeMood.id;
        portEXIT_CRITICAL(&globalDataBusLock);
    } 
}

const char* BehaviourEngine::getActiveModeName() const { return activeMode->getName(); }
const char* BehaviourEngine::getActiveMoodName() const { return "HAPPY"; }

SystemMode BehaviourEngine::mapModeToEnum(IRobotMode* mode) {
    if (mode == normalMode) return SystemMode::MODE_NORMAL_DRIVING;
    if (mode == obstacleMode) return SystemMode::MODE_OBSTACLE_AVOIDANCE;
    if (mode == distanceMode) return SystemMode::MODE_MAINTAIN_DISTANCE;
    if (mode == compassMode) return SystemMode::MODE_COMPASS_LOCK;
    if (mode == dizzyMode) return SystemMode::MODE_DIZZY;
    if (mode == sleepMode) return SystemMode::MODE_DEEP_SLEEP;
    if (mode == teleopMode) return SystemMode::MANUAL_OVERRIDE;
    if (mode == diagnosticMode) return SystemMode::MODE_DIAGNOSTICS;
    if (mode == autotuneMode) return SystemMode::MODE_AUTOTUNE;
    if (mode == brainDeadMode) return SystemMode::MODE_BRAIN_DEAD;
    return SystemMode::MODE_NORMAL_DRIVING;
}

IRobotMode* BehaviourEngine::getModeFromEnum(SystemMode modeEnum) {
    switch(modeEnum) {
        case SystemMode::MODE_NORMAL_DRIVING: return normalMode;
        case SystemMode::MODE_OBSTACLE_AVOIDANCE: return obstacleMode;
        case SystemMode::MODE_MAINTAIN_DISTANCE: return distanceMode;
        case SystemMode::MODE_COMPASS_LOCK: return compassMode;
        case SystemMode::MODE_DIZZY: return dizzyMode;
        case SystemMode::MODE_DEEP_SLEEP: return sleepMode;
        case SystemMode::MANUAL_OVERRIDE: return teleopMode;
        case SystemMode::MODE_DIAGNOSTICS: return diagnosticMode;
        case SystemMode::MODE_AUTOTUNE: return autotuneMode;
        case SystemMode::MODE_BRAIN_DEAD: return brainDeadMode;
        default: return normalMode;
    }
}

void BehaviourEngine::update(const GlobalDataBank& robotData) {
    SystemMode currentSysMode = mapModeToEnum(activeMode);
    SystemMode requestedSysMode = currentSysMode;
    RobotMood requestedMood = activeMood;

    // ==========================================
    // 1. ALWAYS RUN THE PERCEPTION DECIDER 
    // ==========================================
    // By hoisting this to the top, HeuristicDecider calculates deltas and 
    // pushes them to the telemetry bus EVERY loop, regardless of overrides!
    if (decider) {
        decider->evaluate(robotData, currentSysMode, requestedSysMode, requestedMood);
    }

    // ==========================================
    // CHECK DYNAMIC SYSTEM OVERRIDES
    // ==========================================
    // Take a safe snapshot of the teleop bus
    TeleopCommandBus teleopSnapshot;
    portENTER_CRITICAL(&teleopCmdLock);

    // Enforce the 500ms Watchdog Timer so that autonomous brain is resumed
    // if the connection is dropped or the app stops sending control commands
    TeleopCommands.checkFailsafe();
    teleopSnapshot = TeleopCommands;
    portEXIT_CRITICAL(&teleopCmdLock);

    // If the phone is connected via BLE, and Enable Drive is set, we switch to Teleop mode.
    // This ignores whatever Config.BRAIN_ACTIVE is set to.
    // Override Priority A: Manual Teleoperation
    if (teleopSnapshot.isOverrideActive) { 
        changeMode(teleopMode); // Safer internal helper usage
        activeMode->update(activeMood, robotData);
        return; 
    }

    // Override Priority B: The AI/Data Collection Lobotomy where only
    // sensor data is collected and lateches are set/unset
    if (!SysConfig.BRAIN_ACTIVE) {
        changeMode(brainDeadMode);
        activeMode->update(activeMood, robotData);
        return; 
    }

    // ==========================================
    // AUTONOMOUS DECISION EXECUTION
    // ==========================================
    // If no overrides are active, map the Decider's request to hardware
    IRobotMode* nextMode = getModeFromEnum(requestedSysMode);

    if (nextMode != activeMode) {
        changeMode(nextMode); // changeMode() already handles onExit, onEnter, and the Bus lock safely
    }

    // Apply physical hardware constraints based on current mode
    portENTER_CRITICAL(&hardwareCmdLock);
    if (activeMode == normalMode || activeMode == obstacleMode) {
        HardwareCommands.targetFilterBeta = 0.01f;
    } else {
        HardwareCommands.targetFilterBeta = SysConfig.MADGWICK_FILTER_BETA;
    }
    portEXIT_CRITICAL(&hardwareCmdLock);

    // Pulse physical mode loop
    if (activeMode != nullptr) activeMode->update(activeMood, robotData);
}