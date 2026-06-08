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

PerceptionData BehaviourEngine::gatherPerception(const volatile GlobalDataBank& robotData) {
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

void BehaviourEngine::update(const volatile GlobalDataBank& robotData) {

    // THE DYNAMIC OVERRIDE:
    // If the phone is connected via BLE, we immediately force the manual mode.
    // This ignores whatever Config.BRAIN_ACTIVE is set to.
    if (TeleopCommands.isConnected) {
        if (GLOBAL_MODE != SystemMode::MANUAL_OVERRIDE) {
            if (activeMode != nullptr) activeMode->onExit();
            // Assuming 'teleopMode' is available here (pass it via constructor if not)
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
    
    // Pass the physics to the tracker to get clean boolean events!
    SemanticEvents events = latchHandler.processEvents(perception, GLOBAL_MODE);

    IRobotMode* nextMode = determineNextMode(events);

    // ... Determine next mode ...

    if (nextMode != activeMode) {
        if (activeMode != nullptr) activeMode->onExit();
        previousMode = activeMode;
        activeMode = nextMode;
        if (activeMode != nullptr) activeMode->onEnter(robotData); // <--- PASS IT HERE
        GLOBAL_MODE = mapModeToEnum(activeMode); 
    }

    // FIX 3: Use the Command Bus to change the IMU filter, NOT the hardware pointer!
    if (activeMode == normalMode || activeMode == obstacleMode) {
        HardwareCommands.targetFilterBeta = 0.01f;
    } else {
        HardwareCommands.targetFilterBeta = SysConfig.MADGWICK_FILTER_BETA;
    }

    if (activeMode != nullptr) activeMode->update(activeMood, robotData);
}