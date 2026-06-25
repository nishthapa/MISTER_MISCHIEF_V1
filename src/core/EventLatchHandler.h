#pragma once
#include <Arduino.h>
#include "hal/interfaces/I_IMU.h"
#include "core/RobotState.h"

// 1. RAW PHYSICS (What is happening right now?)
struct PerceptionData {
    float currentDistance;
    float distanceDelta;
    float currentGForce;
    float currentYaw;
    float currentPitch;
    float currentRoll;
    float totalRawEnergy;
    float rawYawEnergy;
    float rawPitchEnergy;
    float rawRollEnergy;
    bool isUpright;
    bool isDriving;
    bool hasBarometer;   // <--- NEW
    //float altitudeDelta; replaced by pressureDelta
    float pressureDelta; // <--- Changed from altitudeDelta

};

// 2. SEMANTIC EVENTS (What does the physics mean over time?)
struct SemanticEvents {
    bool hazardDetected;       // Immediate wall detected
    bool teaseConfirmed;       // Hand hovered long enough to play
    bool targetVanished;       // Hand removed safely
    bool dizzyTriggered;       // Shaken too violently
    bool dizzyFinished;        // Recovered from spinning
    bool readyForCompassLock;  // Lifted and held steady
    bool safelyLanded;         // Placed back on the ground
    bool frustrationPeaked;    // Bored of playing the game
};

class EventLatchHandler {
private:
    // Memory State
    float dizzyBarYaw, dizzyBarPitch, dizzyBarRoll;
    float smoothedTotalEnergy;
    float frustrationLevel;
    
    // Active Timers
    unsigned long dizzyStartTime;
    unsigned long teaseStartTime;
    unsigned long vanishingStartTime;
    unsigned long pickupStartTime;
    unsigned long settlingStartTime;

    // Latches
    bool isDriving;
    bool isHandTeasing;
    bool isHandVanishing;
    bool isHandling;
    bool hasExperiencedLift;
    bool isLowering;
    bool hasLanded;
    bool isDizzy;
    
    bool pickupTimerActive;
    bool settlingTimerActive;

public:
    EventLatchHandler();
    void reset();
    
    // Takes the raw physics, updates the timers, and outputs clean True/False events
    SemanticEvents processEvents(const PerceptionData& p, SystemMode currentMode);

    // Getters for internal metrics to populate the Global Data Bus
    float getDizzyBarYaw() const { return dizzyBarYaw; }
    float getDizzyBarPitch() const { return dizzyBarPitch; }
    float getDizzyBarRoll() const { return dizzyBarRoll; }
    float getSmoothedTotalEnergy() const { return smoothedTotalEnergy; }
    float getFrustrationLevel() const { return frustrationLevel; }
    
    bool getIsDriving() const {return isDriving; }
    bool getIsHandTeasing() const { return isHandTeasing; }
    bool getIsHandVanishing() const { return isHandVanishing; }
    bool getIsHandling() const { return isHandling; }
    bool getHasExperiencedLift() const { return hasExperiencedLift; }
    bool getIsLowering() const { return isLowering; }
    bool getHasLanded() const { return hasLanded; }
    bool getIsDizzy() const { return isDizzy; }
};