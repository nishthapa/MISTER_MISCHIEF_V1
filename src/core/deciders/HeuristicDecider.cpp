#include "HeuristicDecider.h"
#include "config/ConfigurationManager.h"
#include "core/GlobalDataBus.h"

extern portMUX_TYPE globalDataBusLock;
extern portMUX_TYPE hardwareCmdLock;

HeuristicDecider::HeuristicDecider(Mode_ObstacleAvoidance* obs) : obstacleMode(obs) {
    latchHandler.reset();
    lastDistance = -1.0f;
    lastAngles = {0, 0, 0, 0, false, 0};
    hasBaro = false;
}

void HeuristicDecider::evaluate(const GlobalDataBank& dataBus, SystemMode currentMode, SystemMode& outRequestedMode, RobotMood& outMood) {
    // 1. Gather raw data
    PerceptionData perception = gatherPerception(dataBus);
    
    // 2. Process into events
    SemanticEvents recentEvents = latchHandler.processEvents(perception, currentMode);
    
    // 3. Decide next mode
    outRequestedMode = determineNextMode(currentMode, recentEvents, dataBus);
    
    // Static for now, as it was in BehaviourEngine
    outMood = Moods::HAPPY; 

    // 4. Safely publish the heuristic-specific metrics directly to the global bus
    portENTER_CRITICAL(&globalDataBusLock);
    
    // Map internal metrics for live telemetry tuning
    CurrentRobotData.perception.distanceDelta = perception.distanceDelta;
    CurrentRobotData.perception.totalRawEnergy = perception.totalRawEnergy;
    CurrentRobotData.perception.rawYawEnergy = perception.rawYawEnergy;
    CurrentRobotData.perception.rawPitchEnergy = perception.rawPitchEnergy;
    CurrentRobotData.perception.rawRollEnergy = perception.rawRollEnergy;
    CurrentRobotData.perception.currentGForce = perception.currentGForce;

    // Map Triggers
    CurrentRobotData.events.hazardDetected = recentEvents.hazardDetected;
    CurrentRobotData.events.teaseConfirmed = recentEvents.teaseConfirmed;
    CurrentRobotData.events.targetVanished = recentEvents.targetVanished;
    CurrentRobotData.events.dizzyTriggered = recentEvents.dizzyTriggered;
    CurrentRobotData.events.dizzyFinished = recentEvents.dizzyFinished;
    CurrentRobotData.events.readyForCompassLock = recentEvents.readyForCompassLock;
    CurrentRobotData.events.safelyLanded = recentEvents.safelyLanded;
    CurrentRobotData.events.frustrationPeaked = recentEvents.frustrationPeaked;

    // Map Internal Metrics
    CurrentRobotData.events.dizzyBarYaw = latchHandler.getDizzyBarYaw();
    CurrentRobotData.events.dizzyBarPitch = latchHandler.getDizzyBarPitch();
    CurrentRobotData.events.dizzyBarRoll = latchHandler.getDizzyBarRoll();
    CurrentRobotData.events.smoothedTotalEnergy = latchHandler.getSmoothedTotalEnergy();
    CurrentRobotData.events.frustrationLevel = latchHandler.getFrustrationLevel();

    // Map Human Interaction Latches
    CurrentRobotData.events.isHandTeasing = latchHandler.getIsHandTeasing();
    CurrentRobotData.events.isHandVanishing = latchHandler.getIsHandVanishing();
    CurrentRobotData.events.isHandling = latchHandler.getIsHandling();
    CurrentRobotData.events.hasExperiencedLift = latchHandler.getHasExperiencedLift();
    CurrentRobotData.events.isLowering = latchHandler.getIsLowering();
    CurrentRobotData.events.hasLanded = latchHandler.getHasLanded();
    CurrentRobotData.events.isDizzy = latchHandler.getIsDizzy();
    
    portEXIT_CRITICAL(&globalDataBusLock);
}

PerceptionData HeuristicDecider::gatherPerception(const GlobalDataBank& robotData) {
    PerceptionData p;
    
    p.hasBarometer = robotData.sensors.hasBaro;
    p.isDriving = robotData.actuators.isDriving;

    static float lastAltitudeCM = 0.0f;
    //p.altitudeDelta = robotData.sensors.altitudeCM - lastAltitudeCM;

    // now working with raw pascal delta for better delta characterization
    p.pressureDelta = robotData.sensors.pressureDeltaPa;

    lastAltitudeCM = robotData.sensors.altitudeCM;

    // 1. Sonar Safety Check
    if (robotData.health.hardwareBitmask & Comms::HealthBit::SONAR_OK) {
        p.currentDistance = robotData.sensors.distanceCM;
    } else {
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

    // 2. IMU Safety Check
    if (robotData.health.hardwareBitmask & Comms::HealthBit::IMU_OK) {
        p.rawYawEnergy = abs(getShortestAngleDelta(robotData.physics.imuAngles.yaw, lastAngles.yaw)) / 0.01f;
        p.rawPitchEnergy = abs(getShortestAngleDelta(robotData.physics.imuAngles.pitch, lastAngles.pitch)) / 0.01f;
        p.rawRollEnergy = abs(getShortestAngleDelta(robotData.physics.imuAngles.roll, lastAngles.roll)) / 0.01f;
        p.totalRawEnergy = p.rawYawEnergy + p.rawPitchEnergy + p.rawRollEnergy;

        p.isUpright = (abs(robotData.physics.imuAngles.pitch) < SysConfig.UPRIGHT_ANGLE_TOLERANCE && abs(robotData.physics.imuAngles.roll) < SysConfig.UPRIGHT_ANGLE_TOLERANCE);

        lastAngles.yaw = robotData.physics.imuAngles.yaw;
        lastAngles.pitch = robotData.physics.imuAngles.pitch;
        lastAngles.roll = robotData.physics.imuAngles.roll;
        lastAngles.gForce = robotData.physics.imuAngles.gForce;
    } else {
        p.currentGForce = 1.0f;
        p.currentYaw = 0.0f;
        p.currentPitch = 0.0f;
        p.currentRoll = 0.0f;

        p.rawYawEnergy = 0.0f;
        p.rawPitchEnergy = 0.0f;
        p.rawRollEnergy = 0.0f;
        p.totalRawEnergy = 0.0f;
        p.isUpright = true;
    }
    lastDistance = p.currentDistance;
    return p;
}

SystemMode HeuristicDecider::determineNextMode(SystemMode activeMode, const SemanticEvents& events, const GlobalDataBank& robotData) {
    bool isDiagnosticRequested = false;
    bool isAutotuneRequested = false;

    portENTER_CRITICAL(&hardwareCmdLock);
    isDiagnosticRequested = HardwareCommands.requestMotorTest;
    isAutotuneRequested = HardwareCommands.requestAutotune;
    portEXIT_CRITICAL(&hardwareCmdLock);

    if (isDiagnosticRequested) return SystemMode::MODE_DIAGNOSTICS;
    if (isAutotuneRequested) return SystemMode::MODE_AUTOTUNE;

    // Priority 3: Survival Sequences (Cannot be interrupted easily)
    if (activeMode == SystemMode::MODE_OBSTACLE_AVOIDANCE) {
        if (obstacleMode && !obstacleMode->isSequenceComplete()) {
            return SystemMode::MODE_OBSTACLE_AVOIDANCE;
        }
    }
    
    if (activeMode == SystemMode::MODE_DIZZY) {
        if (events.dizzyFinished) return SystemMode::MODE_NORMAL_DRIVING;
        return SystemMode::MODE_DIZZY;
    }

    // Priority 4: Handling Interrupts
    if (events.dizzyTriggered) return SystemMode::MODE_DIZZY;
    
    if (activeMode == SystemMode::MODE_COMPASS_LOCK) {
        if (events.safelyLanded) return SystemMode::MODE_NORMAL_DRIVING;
        return SystemMode::MODE_COMPASS_LOCK;
    } else if (events.readyForCompassLock) return SystemMode::MODE_COMPASS_LOCK;

    // Priority 5: The Games
    if (activeMode == SystemMode::MODE_MAINTAIN_DISTANCE) {
        if (events.frustrationPeaked) return SystemMode::MODE_DIZZY;
        if (events.targetVanished) return SystemMode::MODE_NORMAL_DRIVING;
        return SystemMode::MODE_MAINTAIN_DISTANCE;
    }

    if (events.hazardDetected) {
        if (events.teaseConfirmed) return SystemMode::MODE_MAINTAIN_DISTANCE;
        return SystemMode::MODE_OBSTACLE_AVOIDANCE;
    }

    return SystemMode::MODE_NORMAL_DRIVING;
}