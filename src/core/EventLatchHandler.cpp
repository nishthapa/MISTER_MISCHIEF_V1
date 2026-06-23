#include "core/EventLatchHandler.h"
#include "config/ConfigurationManager.h"

EventLatchHandler::EventLatchHandler() { reset(); }

void EventLatchHandler::reset() {
    dizzyBarYaw = 0.0f; dizzyBarPitch = 0.0f; dizzyBarRoll = 0.0f;
    smoothedTotalEnergy = 0.0f; frustrationLevel = 0.0f;
    isDriving = false; isHandTeasing = false; isHandVanishing = false; isHandling = false;
    hasExperiencedLift = false; isLowering = false; hasLanded = false; isDizzy = false;
    pickupTimerActive = false; settlingTimerActive = false;
}

SemanticEvents EventLatchHandler::processEvents(const PerceptionData& p, SystemMode currentMode) {
    SemanticEvents events = {false, false, false, false, false, false, false, false};

    // 1. UPDATE ENERGY & LIFT DETECTORS
    smoothedTotalEnergy = (SysConfig.ENERGY_EMA_ALPHA * p.totalRawEnergy) + (SysConfig.ENERGY_EMA_BETA * smoothedTotalEnergy);
    if (!isHandling && (p.currentGForce > SysConfig.GFORCE_LIFT_UP_THRESHOLD || p.currentGForce < SysConfig.GFORCE_LIFT_DOWN_THRESHOLD || p.totalRawEnergy > SysConfig.LIFT_ENERGY_SPIKE_THRESHOLD)) {
        hasExperiencedLift = true;
    }

    // 2. DIZZY BAR LOGIC
    float effYaw = (currentMode == SystemMode::MODE_OBSTACLE_AVOIDANCE) ? 0.0f : ((p.rawYawEnergy > SysConfig.DIZZY_ENERGY_DEADBAND) ? p.rawYawEnergy : 0.0f);
    float effPitch = (p.rawPitchEnergy > SysConfig.DIZZY_ENERGY_DEADBAND) ? p.rawPitchEnergy : 0.0f;
    float effRoll = (p.rawRollEnergy > SysConfig.DIZZY_ENERGY_DEADBAND) ? p.rawRollEnergy : 0.0f;

    dizzyBarYaw = (SysConfig.DIZZY_CHARGE_RATE * effYaw) + (SysConfig.DIZZY_DECAY_RATE * dizzyBarYaw);
    dizzyBarPitch = (SysConfig.DIZZY_CHARGE_RATE * effPitch) + (SysConfig.DIZZY_DECAY_RATE * dizzyBarPitch);
    dizzyBarRoll = (SysConfig.DIZZY_CHARGE_RATE * effRoll) + (SysConfig.DIZZY_DECAY_RATE * dizzyBarRoll);

    if ((dizzyBarYaw > SysConfig.DIZZY_TRIGGER_THRESHOLD || dizzyBarPitch > SysConfig.DIZZY_TRIGGER_THRESHOLD || dizzyBarRoll > SysConfig.DIZZY_TRIGGER_THRESHOLD) && !isDizzy) {
        isDizzy = true; dizzyStartTime = millis(); dizzyBarYaw = 0.0f; dizzyBarPitch = 0.0f; dizzyBarRoll = 0.0f;
        events.dizzyTriggered = true;
    }
    if (isDizzy && millis() - dizzyStartTime > SysConfig.DIZZY_DURATION_MS) {
        isDizzy = false; reset(); events.dizzyFinished = true;
    }

    // 3. DISTANCE / OBSTACLE LOGIC
    events.hazardDetected = (p.currentDistance > 0 && p.currentDistance < SysConfig.OBSTACLE_TRIGGER_CM);

    if (events.hazardDetected) {
        if (p.distanceDelta < -15.0f && !isHandTeasing) { isHandTeasing = true; teaseStartTime = millis(); }
        if (isHandTeasing && millis() - teaseStartTime > 700) events.teaseConfirmed = true;
    } else { isHandTeasing = false; }

    if (currentMode == SystemMode::MODE_MAINTAIN_DISTANCE) {
        frustrationLevel += SysConfig.FRUSTRATION_HEATUP_RATE;
        if (p.distanceDelta > 20.0f) { isHandVanishing = true; vanishingStartTime = millis(); }
        if (isHandVanishing) {
            if (p.distanceDelta < -15.0f) isHandVanishing = false; 
            else if (millis() - vanishingStartTime > 700) { events.targetVanished = true; isHandVanishing = false; isHandTeasing = false; }
        } else if (p.currentDistance > 80.0f) { events.targetVanished = true; isHandVanishing = false; isHandTeasing = false; }
    } else { frustrationLevel -= SysConfig.FRUSTRATION_COOLDOWN_RATE; }

    if (frustrationLevel < 0.0f) frustrationLevel = 0.0f;
    if (frustrationLevel >= SysConfig.DISTANCE_HOLD_FRUSTRATION_LIMIT) { events.frustrationPeaked = true; frustrationLevel = 0.0f; }

    // 4. HANDLING & COMPASS LOCK LOGIC
    if (!p.isUpright) { if (hasExperiencedLift) isHandling = true; }
    //if (p.isUpright && smoothedTotalEnergy < SysConfig.PERFECTLY_STILL_ENERGY) isHandling = false;

    if (p.isUpright && (smoothedTotalEnergy < SysConfig.PERFECTLY_STILL_ENERGY || p.isDriving)) {
        isHandling = false;
        hasExperiencedLift = false;
    }

    // if (currentMode != SystemMode::MODE_COMPASS_LOCK) {
    //     if (isHandling && p.totalRawEnergy < SysConfig.STEADY_HOLD_ENERGY_MAX) {
    //         if (!pickupTimerActive) { pickupTimerActive = true; pickupStartTime = millis(); } 
    //         else if (millis() - pickupStartTime >= SysConfig.COMPASS_LOCK_ENTRY_SETTLE_MS) {
    //             pickupTimerActive = false; settlingTimerActive = false; isLowering = false; hasLanded = false; 
    //             events.readyForCompassLock = true;
    //         }
    //     } else { pickupTimerActive = false; }
    // } else {

    if (currentMode != SystemMode::MODE_COMPASS_LOCK) {
        // Only enter compass lock if we are actually being handled, held steady, AND NOT DRIVING
        if (isHandling && !p.isDriving && p.totalRawEnergy < SysConfig.STEADY_HOLD_ENERGY_MAX) {
            if (!pickupTimerActive) { pickupTimerActive = true; pickupStartTime = millis(); } 
            else if (millis() - pickupStartTime >= SysConfig.COMPASS_LOCK_ENTRY_SETTLE_MS) {
                    pickupTimerActive = false; settlingTimerActive = false; isLowering = false; hasLanded = false; 
                    events.readyForCompassLock = true;
                }
            } else { pickupTimerActive = false; }
        } else {

        if (p.currentGForce < SysConfig.GFORCE_LIFT_DOWN_THRESHOLD) { isLowering = true; hasLanded = false; settlingTimerActive = false; }
        if (isLowering && (p.currentGForce > SysConfig.GFORCE_LIFT_UP_THRESHOLD || p.totalRawEnergy > SysConfig.STEADY_HOLD_ENERGY_MAX)) { hasLanded = true; isLowering = false; }
        if (hasLanded || (!isHandling && smoothedTotalEnergy < SysConfig.PERFECTLY_STILL_ENERGY)) {
            if (p.isUpright && smoothedTotalEnergy < SysConfig.PERFECTLY_STILL_ENERGY) {
                if (!settlingTimerActive) { settlingTimerActive = true; settlingStartTime = millis(); } 
                else if (millis() - settlingStartTime >= SysConfig.COMPASS_LOCK_EXIT_SETTLE_MS) {
                    isHandling = false; hasExperiencedLift = false; isLowering = false; hasLanded = false; settlingTimerActive = false; 
                    events.safelyLanded = true;
                }
            } else { settlingTimerActive = false; }
        } else { settlingTimerActive = false; }
    }
    return events;
}