#include "core/BehaviourEngine.h"
//#include "config/PersonalityConfig.h" // Safe to DELETE since we're now pulling these values from the NVS-backed ConfigurationManager
// #include "config/BehaviourEngineConfig.h" // <-- The Tuning Dashboard!
#include <Arduino.h>

// ==========================================
// MODE BLUEPRINTS so the compiler knows these inherit from IRobotMode!
// ==========================================
#include "behaviours/Mode_ObstacleAvoidance.h"
#include "behaviours/Mode_NormalDriving.h"
#include "behaviours/Mode_CompassLock.h"
#include "behaviours/Mode_MaintainDistance.h"
#include "behaviours/Mode_Dizzy.h"
#include "behaviours/Mode_DeepSleep.h"
#include "config/ConfigurationManager.h" // <-- For the master clock and personality parameters

BehaviourEngine::BehaviourEngine(I_IMU* i, I_DistanceSensor* s, 
                                 Mode_ObstacleAvoidance* obs, Mode_NormalDriving* norm, 
                                 Mode_CompassLock* comp, Mode_MaintainDistance* dist, 
                                 Mode_Dizzy* diz, Mode_DeepSleep* sleep) {
    imu = i; sonar = s;
    obstacleMode = obs; normalMode = norm; compassMode = comp;
    distanceMode = dist; dizzyMode = diz; sleepMode = sleep;
    
    activeMode = normalMode;
    previousMode = nullptr;
    activeMood = Moods::HAPPY;

    frustrationLevel = 0.0f;
    isGroggyPhase = false;
    isDizzy = false;
    isHandling = false;
    hasExperiencedLift = false;
    
    // Initialize our latches!
    isLowering = false;
    hasLanded = false;
    
    dizzyBarYaw = 0.0f; dizzyBarPitch = 0.0f; dizzyBarRoll = 0.0f;
    smoothedTotalEnergy = 0.0f;
    lastDistance = -1.0f;
    lastAngles = {0,0,0,0,false,0};
}

void BehaviourEngine::init(bool isColdBoot) {
    if (isColdBoot) {
        activeMood = Moods::GROGGY;
        isGroggyPhase = true;
        coldBootTime = millis();
    } else {
        activeMood = Moods::HAPPY;
        isGroggyPhase = false; 
    }
    activeMode->onEnter();
    previousMode = activeMode;
}

const char* BehaviourEngine::getActiveModeName() const {
    return activeMode->getName();
}

const char* BehaviourEngine::getActiveMoodName() const {
    if (activeMood.speedMultiplier == Moods::GROGGY.speedMultiplier) return "GROGGY";
    if (activeMood.speedMultiplier == Moods::ANGRY.speedMultiplier) return "ANGRY";
    return "HAPPY";
}

void BehaviourEngine::update() {
    FusedAngles currentAngles = imu->getAngles();
    float distance = sonar->getDistanceCM();
    
    float distanceDelta = (distance != lastDistance && lastDistance > 0.0f) ? (distance - lastDistance) : 0.0f;
    lastDistance = distance;
    
    auto getShortestAngleDelta = [](float current, float previous) {
        float delta = current - previous;
        if (delta > 180.0f) delta -= 360.0f;
        else if (delta < -180.0f) delta += 360.0f;
        return delta;
    };
    
    float rawYawEnergy = abs(getShortestAngleDelta(currentAngles.yaw, lastAngles.yaw)) / 0.01f;
    float rawPitchEnergy = abs(getShortestAngleDelta(currentAngles.pitch, lastAngles.pitch)) / 0.01f;
    float rawRollEnergy = abs(getShortestAngleDelta(currentAngles.roll, lastAngles.roll)) / 0.01f;
    
    float totalRawEnergy = rawYawEnergy + rawPitchEnergy + rawRollEnergy;
    smoothedTotalEnergy = (Config.ENERGY_EMA_ALPHA * totalRawEnergy) + (Config.ENERGY_EMA_BETA * smoothedTotalEnergy);

    // ==========================================
    // THE PHYSICS LATCH FIX
    // ==========================================
    // ONLY check for lift G-forces if he isn't already being handled!
    float currentGForce = currentAngles.gForce;
    if (!isHandling) {
        if (currentGForce > Config.GFORCE_LIFT_UP_THRESHOLD || 
            currentGForce < Config.GFORCE_LIFT_DOWN_THRESHOLD || 
            totalRawEnergy > Config.LIFT_ENERGY_SPIKE_THRESHOLD) {
            hasExperiencedLift = true;
        }
    }

    float effectiveYawEnergy = (rawYawEnergy > Config.DIZZY_ENERGY_DEADBAND) ? rawYawEnergy : 0.0f;
    float effectivePitchEnergy = (rawPitchEnergy > Config.DIZZY_ENERGY_DEADBAND) ? rawPitchEnergy : 0.0f;
    float effectiveRollEnergy = (rawRollEnergy > Config.DIZZY_ENERGY_DEADBAND) ? rawRollEnergy : 0.0f;
    
    dizzyBarYaw = (Config.DIZZY_CHARGE_RATE * effectiveYawEnergy) + (Config.DIZZY_DECAY_RATE * dizzyBarYaw);
    dizzyBarPitch = (Config.DIZZY_CHARGE_RATE * effectivePitchEnergy) + (Config.DIZZY_DECAY_RATE * dizzyBarPitch);
    dizzyBarRoll = (Config.DIZZY_CHARGE_RATE * effectiveRollEnergy) + (Config.DIZZY_DECAY_RATE * dizzyBarRoll);

    lastAngles = currentAngles;
    
    // ==========================================
    // THE DECISION TREE
    // ==========================================
    // PRIORITY 1A: EMERGENCY & SPATIAL AWARENESS
    if (activeMode == obstacleMode && !obstacleMode->isSequenceComplete()) {
        // === THE FIX 13: THE "ABORT ON LIFT" BREAKOUT ===
        // If he is fighting an obstacle but suddenly gets yanked into the air,
        // we must ABORT the obstacle sequence so Priority 3 can take over!
        if (hasExperiencedLift || isHandling) {
            // Do nothing here. This allows the code to fall through to Priority 3.
            // The Transition Manager will automatically call obstacleMode->onExit() to kill the tracks!
        } else {
            activeMode = obstacleMode;
        }
    }
    else if (distance > 0 && distance < Config.OBSTACLE_TRIGGER_CM) {
        if (activeMode == distanceMode) activeMode = distanceMode;
        else if (distanceDelta < -15.0f) { activeMode = distanceMode; isDizzy = false; }
        else { activeMode = obstacleMode; isDizzy = false; }
    } 
    else if (activeMode == distanceMode && distance > 0 && distance < (Config.OBSTACLE_TRIGGER_CM + 15.0f)) {
        activeMode = distanceMode;
    }
    else if ((dizzyBarYaw > Config.DIZZY_TRIGGER_THRESHOLD || 
              dizzyBarPitch > Config.DIZZY_TRIGGER_THRESHOLD || 
              dizzyBarRoll > Config.DIZZY_TRIGGER_THRESHOLD) && !isDizzy) {
        isDizzy = true;
        dizzyStartTime = millis(); activeMode = dizzyMode;
        dizzyBarYaw = 0.0f; dizzyBarPitch = 0.0f; dizzyBarRoll = 0.0f;
    }
    else if (isDizzy) {
        if (millis() - dizzyStartTime > Config.DIZZY_DURATION_MS) {
            isDizzy = false;
            isHandling = false; hasExperiencedLift = false; 
            isLowering = false; hasLanded = false; // Reset gravity sequence memory!
            pickupTimerActive = false; settlingTimerActive = false;
            if (activeMode == dizzyMode) activeMode = normalMode;
        } else {
            activeMode = dizzyMode;
        }
    }
    else {
        // THE HANDLING LATCH
        if (abs(currentAngles.pitch) > Config.TILT_HANDLING_THRESHOLD || 
            abs(currentAngles.roll) > Config.TILT_HANDLING_THRESHOLD) {
            if (hasExperiencedLift) isHandling = true;
        }

        bool isUpright = (abs(currentAngles.pitch) < Config.UPRIGHT_ANGLE_TOLERANCE && 
                          abs(currentAngles.roll) < Config.UPRIGHT_ANGLE_TOLERANCE);
                          
        // Restored to the correct math boundary so he actually knows when he is put down!
        if (isUpright && smoothedTotalEnergy < Config.PERFECTLY_STILL_ENERGY) {
            isHandling = false;
        }

        if (activeMode != compassMode) {
            if (isHandling) {
                if (totalRawEnergy < Config.STEADY_HOLD_ENERGY_MAX) {
                    if (!pickupTimerActive) {
                        pickupTimerActive = true; pickupStartTime = millis();
                    } else if (millis() - pickupStartTime >= Config.COMPASS_LOCK_ENTRY_SETTLE_MS) {
                        activeMode = compassMode;
                        pickupTimerActive = false; 
                        settlingTimerActive = false; 
                        isLowering = false; hasLanded = false; // Prepare the exit sequence variables!
                    }
                } else { pickupTimerActive = false; }
            } else { pickupTimerActive = false; }
        }
        else if (activeMode == compassMode) {
            // ========================================================
            // THE USER'S GRAVITY EXIT SEQUENCE (Restored & Config-Ready)
            // ========================================================
            
            // 1. Detect lowering (Partial Freefall)
            if (currentGForce < Config.GFORCE_LIFT_DOWN_THRESHOLD) {
                isLowering = true;
                hasLanded = false;
                settlingTimerActive = false;
            }
            
            // 2. Detect Impact (High G or Energy Spike) while lowering
            if (isLowering && (currentGForce > Config.GFORCE_LIFT_UP_THRESHOLD || totalRawEnergy > Config.STEADY_HOLD_ENERGY_MAX)) {
                hasLanded = true;
                isLowering = false;
            }

            // 3. The Table Test (Has Landed OR Gently Placed down with math satisfaction)
            if (hasLanded || (!isHandling && smoothedTotalEnergy < Config.PERFECTLY_STILL_ENERGY)) {
                if (isUpright && smoothedTotalEnergy < Config.PERFECTLY_STILL_ENERGY) {
                    if (!settlingTimerActive) {
                        settlingTimerActive = true;
                        settlingStartTime = millis();
                    } else if (millis() - settlingStartTime >= Config.COMPASS_LOCK_EXIT_SETTLE_MS) {
                        // Success! Safely on the ground.
                        activeMode = normalMode;
                        isHandling = false;
                        hasExperiencedLift = false;
                        isLowering = false;
                        hasLanded = false;
                        settlingTimerActive = false; 
                    }
                } else {
                    // Wiggling! Reset the timer.
                    settlingTimerActive = false;
                }
            } else {
                // Not landed yet.
                settlingTimerActive = false;
            }
        }
        else { activeMode = normalMode; }
    } 

    // ==========================================
    // THE MOOD ENGINE
    // ==========================================
    if (activeMode == distanceMode) frustrationLevel += Config.FRUSTRATION_HEATUP_RATE;
    else frustrationLevel -= Config.FRUSTRATION_COOLDOWN_RATE; 

    if (frustrationLevel < 0.0f) frustrationLevel = 0.0f;
    if (frustrationLevel > Config.DISTANCE_HOLD_FRUSTRATION_LIMIT + 50.0f) {
        frustrationLevel = Config.DISTANCE_HOLD_FRUSTRATION_LIMIT + 50.0f;
    }

    if (frustrationLevel >= Config.DISTANCE_HOLD_FRUSTRATION_LIMIT) activeMood = Moods::ANGRY;
    else activeMood = Moods::HAPPY;
    
    if (isGroggyPhase) {
        if (millis() - coldBootTime > Moods::GROGGY_DURATION_MS) isGroggyPhase = false;
        else activeMood = Moods::GROGGY; 
    }

    // ==========================================
    // THE TRANSITION MANAGER
    // ==========================================
    if (activeMode != previousMode) {
        if (previousMode != nullptr) previousMode->onExit();
        if (activeMode != nullptr) activeMode->onEnter();
        previousMode = activeMode;
    }

    // --- ADAPTIVE IMU FILTERING ---
    // If the tracks are actively moving, drop the filter beta to ignore vibration
    if (activeMode == normalMode || activeMode == obstacleMode) {
        imu->setFilterBeta(0.01f); // Stiff gyro trust
    } else {
        // When stopped (Compass Lock, Deep Sleep, Handling), use the live tuned Config beta to re-level quickly
        imu->setFilterBeta(Config.MADGWICK_FILTER_BETA); 
    }
    // ----------------------------------------------

    if (activeMode != nullptr) activeMode->update(activeMood);
}