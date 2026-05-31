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

    isHandTeasing = false; ambientBackgroundDistance = 400.0f; 
    activeMode = normalMode; previousMode = nullptr; activeMood = Moods::HAPPY;
    frustrationLevel = 0.0f; isGroggyPhase = false; isDizzy = false;
    isHandling = false; hasExperiencedLift = false; isLowering = false; hasLanded = false;
    isHandVanishing = false; dizzyBarYaw = 0.0f; dizzyBarPitch = 0.0f; dizzyBarRoll = 0.0f;
    smoothedTotalEnergy = 0.0f; lastDistance = -1.0f; lastAngles = {0,0,0,0,false,0};
}

void BehaviourEngine::init(bool isColdBoot) {
    if (isColdBoot) {
        activeMood = Moods::GROGGY; isGroggyPhase = true; coldBootTime = millis();
    } else {
        activeMood = Moods::HAPPY; isGroggyPhase = false; 
    }
    GLOBAL_MODE = mapModeToEnum(activeMode);
    activeMode->onEnter();
    previousMode = activeMode;
}

void BehaviourEngine::changeMode(IRobotMode* newMode) {
    if (newMode != nullptr) activeMode = newMode;
}

const char* BehaviourEngine::getActiveModeName() const { return activeMode->getName(); }

const char* BehaviourEngine::getActiveMoodName() const {
    if (activeMood.speedMultiplier == Moods::GROGGY.speedMultiplier) return "GROGGY";
    if (activeMood.speedMultiplier == Moods::ANGRY.speedMultiplier) return "ANGRY";
    return "HAPPY";
}

SystemMode BehaviourEngine::mapModeToEnum(IRobotMode* mode) {
    if (mode == normalMode) return SystemMode::MODE_NORMAL_DRIVING;
    if (mode == obstacleMode) return SystemMode::MODE_OBSTACLE_AVOIDANCE;
    if (mode == distanceMode) return SystemMode::MODE_MAINTAIN_DISTANCE;
    if (mode == compassMode) return SystemMode::MODE_COMPASS_LOCK;
    if (mode == dizzyMode) return SystemMode::MODE_DIZZY;
    if (mode == sleepMode) return SystemMode::MODE_DEEP_SLEEP;
    return SystemMode::MODE_NORMAL_DRIVING; // Fallback
}

// ==========================================
// 1. PERCEPTION: GATHER ALL MENTAL MATH
// ==========================================
PerceptionData BehaviourEngine::gatherPerception() {
    PerceptionData p;
    FusedAngles currentAngles = imu->getAngles();
    p.currentDistance = sonar->getDistanceCM();
    p.distanceDelta = (p.currentDistance != lastDistance && lastDistance > 0.0f) ? (p.currentDistance - lastDistance) : 0.0f;
    p.currentGForce = currentAngles.gForce;

    auto getShortestAngleDelta = [](float current, float previous) {
        float delta = current - previous;
        if (delta > 180.0f) delta -= 360.0f;
        else if (delta < -180.0f) delta += 360.0f;
        return delta;
    };
    
    p.rawYawEnergy = abs(getShortestAngleDelta(currentAngles.yaw, lastAngles.yaw)) / 0.01f;
    p.rawPitchEnergy = abs(getShortestAngleDelta(currentAngles.pitch, lastAngles.pitch)) / 0.01f;
    p.rawRollEnergy = abs(getShortestAngleDelta(currentAngles.roll, lastAngles.roll)) / 0.01f;
    p.totalRawEnergy = p.rawYawEnergy + p.rawPitchEnergy + p.rawRollEnergy;
    p.isUpright = (abs(currentAngles.pitch) < Config.UPRIGHT_ANGLE_TOLERANCE && abs(currentAngles.roll) < Config.UPRIGHT_ANGLE_TOLERANCE);

    // Update internal rolling averages
    smoothedTotalEnergy = (Config.ENERGY_EMA_ALPHA * p.totalRawEnergy) + (Config.ENERGY_EMA_BETA * smoothedTotalEnergy);
    
    if (!isHandling && (p.currentGForce > Config.GFORCE_LIFT_UP_THRESHOLD || p.currentGForce < Config.GFORCE_LIFT_DOWN_THRESHOLD || p.totalRawEnergy > Config.LIFT_ENERGY_SPIKE_THRESHOLD)) {
        hasExperiencedLift = true;
    }

    float effYaw = (activeMode == obstacleMode) ? 0.0f : ((p.rawYawEnergy > Config.DIZZY_ENERGY_DEADBAND) ? p.rawYawEnergy : 0.0f);
    float effPitch = (p.rawPitchEnergy > Config.DIZZY_ENERGY_DEADBAND) ? p.rawPitchEnergy : 0.0f;
    float effRoll = (p.rawRollEnergy > Config.DIZZY_ENERGY_DEADBAND) ? p.rawRollEnergy : 0.0f;

    dizzyBarYaw = (Config.DIZZY_CHARGE_RATE * effYaw) + (Config.DIZZY_DECAY_RATE * dizzyBarYaw);
    dizzyBarPitch = (Config.DIZZY_CHARGE_RATE * effPitch) + (Config.DIZZY_DECAY_RATE * dizzyBarPitch);
    dizzyBarRoll = (Config.DIZZY_CHARGE_RATE * effRoll) + (Config.DIZZY_DECAY_RATE * dizzyBarRoll);

    lastAngles = currentAngles;
    lastDistance = p.currentDistance;

    return p;
}

// ==========================================
// 2. MOOD ENGINE
// ==========================================
void BehaviourEngine::updateMoodTracker(const PerceptionData& p) {
    if (activeMode == distanceMode) frustrationLevel += Config.FRUSTRATION_HEATUP_RATE;
    else frustrationLevel -= Config.FRUSTRATION_COOLDOWN_RATE; 

    if (frustrationLevel < 0.0f) frustrationLevel = 0.0f;
    if (frustrationLevel > Config.DISTANCE_HOLD_FRUSTRATION_LIMIT + 50.0f) frustrationLevel = Config.DISTANCE_HOLD_FRUSTRATION_LIMIT + 50.0f;

    if (frustrationLevel >= Config.DISTANCE_HOLD_FRUSTRATION_LIMIT) activeMood = Moods::ANGRY;
    else activeMood = Moods::HAPPY;
    
    if (isGroggyPhase) {
        if (millis() - coldBootTime > Moods::GROGGY_DURATION_MS) isGroggyPhase = false;
        else activeMood = Moods::GROGGY; 
    }
}

// ==========================================
// 3. DECISION ENGINE (The Switchboard)
// ==========================================
IRobotMode* BehaviourEngine::determineNextMode(const PerceptionData& p) {
    
    // --- 1. EMERGENCY AVOIDANCE ---
    if (activeMode == obstacleMode && !obstacleMode->isSequenceComplete()) {
        if (hasExperiencedLift || isHandling) return normalMode; // Abort out if lifted
        return obstacleMode; 
    }

    // --- 2. MAINTAIN DISTANCE GAME ---
    if (activeMode == distanceMode) {
        if (frustrationLevel >= Config.DISTANCE_HOLD_FRUSTRATION_LIMIT) {
            isDizzy = true; dizzyStartTime = millis(); isHandVanishing = false;
            return dizzyMode;
        }
        if (p.distanceDelta > 20.0f) { isHandVanishing = true; vanishingStartTime = millis(); }
        if (isHandVanishing) {
            if (p.distanceDelta < -15.0f) isHandVanishing = false; // Cancel exit
            else if (millis() - vanishingStartTime > 700) { isHandVanishing = false; isHandTeasing = false; return normalMode; }
        } else if (p.currentDistance > 80.0f) {
            isHandVanishing = false; isHandTeasing = false; return normalMode;
        }
        return distanceMode;
    }

    // --- 3. TRIGGERING THE GAME VS OBSTACLE ---
    if (p.currentDistance > 0 && p.currentDistance < Config.OBSTACLE_TRIGGER_CM) {
        if (p.distanceDelta < -15.0f && !isHandTeasing) { 
            isHandTeasing = true; teaseStartTime = millis();
        }
        if (isHandTeasing) {
            if (millis() - teaseStartTime > 700) { isDizzy = false; return distanceMode; }
            return normalMode; // Waiting for tease to confirm
        } else { 
            isDizzy = false; return obstacleMode; 
        }
    }

    // --- 4. THE DIZZY REFLEX ---
    if ((dizzyBarYaw > Config.DIZZY_TRIGGER_THRESHOLD || dizzyBarPitch > Config.DIZZY_TRIGGER_THRESHOLD || dizzyBarRoll > Config.DIZZY_TRIGGER_THRESHOLD) && !isDizzy) {
        isDizzy = true; dizzyStartTime = millis(); dizzyBarYaw = 0.0f; dizzyBarPitch = 0.0f; dizzyBarRoll = 0.0f;
        return dizzyMode;
    }
    if (isDizzy) {
        if (millis() - dizzyStartTime > Config.DIZZY_DURATION_MS) {
            isDizzy = false; isHandling = false; hasExperiencedLift = false; 
            isLowering = false; hasLanded = false; pickupTimerActive = false; settlingTimerActive = false;
            return normalMode;
        }
        return dizzyMode;
    }

    // --- 5. HANDLING & COMPASS LOCK ---
    if (abs(lastAngles.pitch) > Config.TILT_HANDLING_THRESHOLD || abs(lastAngles.roll) > Config.TILT_HANDLING_THRESHOLD) {
        if (hasExperiencedLift) isHandling = true;
    }
    if (p.isUpright && smoothedTotalEnergy < Config.PERFECTLY_STILL_ENERGY) isHandling = false;

    if (activeMode != compassMode) {
        if (isHandling) {
            if (p.totalRawEnergy < Config.STEADY_HOLD_ENERGY_MAX) {
                if (!pickupTimerActive) { pickupTimerActive = true; pickupStartTime = millis(); } 
                else if (millis() - pickupStartTime >= Config.COMPASS_LOCK_ENTRY_SETTLE_MS) {
                    pickupTimerActive = false; settlingTimerActive = false; isLowering = false; hasLanded = false; 
                    return compassMode;
                }
            } else { pickupTimerActive = false; }
        } else { pickupTimerActive = false; }
    } else {
        // Exiting Compass Lock
        if (p.currentGForce < Config.GFORCE_LIFT_DOWN_THRESHOLD) { isLowering = true; hasLanded = false; settlingTimerActive = false; }
        if (isLowering && (p.currentGForce > Config.GFORCE_LIFT_UP_THRESHOLD || p.totalRawEnergy > Config.STEADY_HOLD_ENERGY_MAX)) { hasLanded = true; isLowering = false; }
        if (hasLanded || (!isHandling && smoothedTotalEnergy < Config.PERFECTLY_STILL_ENERGY)) {
            if (p.isUpright && smoothedTotalEnergy < Config.PERFECTLY_STILL_ENERGY) {
                if (!settlingTimerActive) { settlingTimerActive = true; settlingStartTime = millis(); } 
                else if (millis() - settlingStartTime >= Config.COMPASS_LOCK_EXIT_SETTLE_MS) {
                    isHandling = false; hasExperiencedLift = false; isLowering = false; hasLanded = false; settlingTimerActive = false; 
                    return normalMode;
                }
            } else { settlingTimerActive = false; }
        } else { settlingTimerActive = false; }
        return compassMode;
    }

    return normalMode;
}

// ==========================================
// 4. THE MAIN LOOP (Clean & Predictable)
// ==========================================
void BehaviourEngine::update() {
    if (!Config.BRAIN_ACTIVE) {
        GLOBAL_MODE = SystemMode::MANUAL_OVERRIDE;
        if (activeMode) activeMode->update(activeMood);
        return;
    }

    // 1. Math
    PerceptionData perception = gatherPerception();
    updateMoodTracker(perception);

    // 2. Decide
    IRobotMode* nextMode = determineNextMode(perception);

    // 3. Switch (If needed)
    if (nextMode != activeMode) {
        if (activeMode != nullptr) activeMode->onExit();
        previousMode = activeMode;
        activeMode = nextMode;
        if (activeMode != nullptr) activeMode->onEnter();
        
        GLOBAL_MODE = mapModeToEnum(activeMode); // Update System Variable
    }

    // 4. Adapt Filters
    if (activeMode == normalMode || activeMode == obstacleMode) imu->setFilterBeta(0.01f);
    else imu->setFilterBeta(Config.MADGWICK_FILTER_BETA); 

    // 5. Execute
    if (activeMode != nullptr) activeMode->update(activeMood);
}