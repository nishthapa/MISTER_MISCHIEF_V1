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
#include "config/SystemConfig.h"

BehaviourEngine::BehaviourEngine(Mode_ObstacleAvoidance* obs, Mode_NormalDriving* norm, 
                                Mode_CompassLock* comp, Mode_MaintainDistance* dist, 
                                Mode_Dizzy* diz, Mode_DeepSleep* sleep, Mode_Teleop* teleop,
                                Mode_Diagnostics* diag, Mode_AutoTune* autot,
                                Mode_BrainDead* deadMode) {

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
    // 1. DETERMINISTIC & AI PERCEPTION (Handled entirely by LatchHandler)
    // ==========================================
    SemanticEvents deterministicEvents = latchHandler.processEvents(robotData);
    
    portENTER_CRITICAL(&globalDataBusLock);
    CurrentRobotData.events.isHandling = deterministicEvents.isHandling;
    CurrentRobotData.events.isFreeFalling = deterministicEvents.isFreeFalling;
    CurrentRobotData.events.isAbsolutelyStill = deterministicEvents.isAbsolutelyStill;
    CurrentRobotData.events.isUpsideDown = deterministicEvents.isUpsideDown;
    CurrentRobotData.events.isTippedLeft = deterministicEvents.isTippedLeft;
    CurrentRobotData.events.isTippedRight = deterministicEvents.isTippedRight;
    CurrentRobotData.events.isNoseUp = deterministicEvents.isNoseUp;
    CurrentRobotData.events.isNoseDown = deterministicEvents.isNoseDown;
    
    // Push the neural network outputs directly into the global data bus
    CurrentRobotData.events.hazardDetected = deterministicEvents.hazardDetected;
    CurrentRobotData.events.isImpactDetected = deterministicEvents.isImpactDetected;

    // NEW: Push the lift latch to the global bus
    CurrentRobotData.events.hasExperiencedLift = deterministicEvents.hasExperiencedLift;
    
    CurrentRobotData.perception.smoothedTotalEnergy = deterministicEvents.smoothedTotalEnergy;
    portEXIT_CRITICAL(&globalDataBusLock);

    // ==========================================
    // 2. THE STATE MACHINE (The "Brain")
    // ==========================================
    portENTER_CRITICAL(&hardwareCmdLock);
    bool isDiag = HardwareCommands.requestMotorTest;
    bool isTune = HardwareCommands.requestAutotune;
    portEXIT_CRITICAL(&hardwareCmdLock);

    if (isDiag) requestedSysMode = SystemMode::MODE_DIAGNOSTICS;
    else if (isTune) requestedSysMode = SystemMode::MODE_AUTOTUNE;
    
    else if (currentSysMode == SystemMode::MODE_OBSTACLE_AVOIDANCE && !obstacleMode->isSequenceComplete()) {
        requestedSysMode = SystemMode::MODE_OBSTACLE_AVOIDANCE;
    }
    else if (CurrentRobotData.events.isHandling) {
        //requestedSysMode = SystemMode::MODE_COMPASS_LOCK;
        // Kill motors if its being handled by a human
        requestedSysMode = SystemMode::MODE_BRAIN_DEAD;
    }
    else if (CurrentRobotData.events.isUpsideDown) {
        requestedSysMode = SystemMode::MODE_DIZZY; 
    }
    else if (CurrentRobotData.events.isStuck || CurrentRobotData.events.hazardDetected) {
        requestedSysMode = SystemMode::MODE_OBSTACLE_AVOIDANCE;
    }
    else {
        requestedSysMode = SystemMode::MODE_NORMAL_DRIVING;
    }

    // ==========================================
    // 3. DYNAMIC OVERRIDES (Teleop / BrainDead)
    // ==========================================
    TeleopCommandBus teleopSnapshot;
    portENTER_CRITICAL(&teleopCmdLock);
    TeleopCommands.checkFailsafe();
    teleopSnapshot = TeleopCommands;
    portEXIT_CRITICAL(&teleopCmdLock);

    if (teleopSnapshot.isOverrideActive) { 
        changeMode(teleopMode);
        activeMode->update(activeMood, robotData);
        return;
    }
    if (!SysConfig.BRAIN_ACTIVE) {
        changeMode(brainDeadMode);
        activeMode->update(activeMood, robotData);
        return; 
    }

    // ==========================================
    // 4. EXECUTE AUTONOMOUS MODE
    // ==========================================
    IRobotMode* nextMode = getModeFromEnum(requestedSysMode);
    if (nextMode != activeMode) {
        changeMode(nextMode);
    }

    portENTER_CRITICAL(&hardwareCmdLock);
    HardwareCommands.targetFilterBeta = (activeMode == normalMode || activeMode == obstacleMode) ? 0.01f : SysConfig.MADGWICK_FILTER_BETA;
    portEXIT_CRITICAL(&hardwareCmdLock);

    if (activeMode != nullptr) activeMode->update(activeMood, robotData);
}

