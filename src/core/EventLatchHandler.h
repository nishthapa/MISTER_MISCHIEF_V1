#pragma once
#include <Arduino.h>
#include "hal/interfaces/I_IMU.h"
#include "core/RobotState.h"
#include "core/GlobalDataBus.h" // <--- FIX 1: Now it knows what GlobalDataBank is!

// 1. SEMANTIC EVENTS (The Physical Truths + AI Latches)
struct SemanticEvents {
    // --- DETERMINISTIC STATES (Calculated in C++) ---
    bool isHandling = false;
    bool isFreeFalling = false;
    bool isAbsolutelyStill = false;
    
    // Orientations
    bool isUpright = true;
    bool isUpsideDown = false;
    bool isTippedLeft = false;
    bool isTippedRight = false;
    bool isNoseUp = false;
    bool isNoseDown = false;

    // --- AI STATES (Calculated later by the Neural Network) ---
    bool isStuck = false;           
    bool hazardDetected = false;    
    bool isBeingTeased = false;     
    bool isBeingPushed = false;

    float smoothedTotalEnergy = 0.0f;
};

class EventLatchHandler {
private:
    // Only the memory strictly needed for the Pre-AI filter!
    float smoothedTotalEnergy;
    bool isHandling;

    float lastDistance;
    FusedAngles lastAngles;
    
public:
    EventLatchHandler();
    
    // <--- FIX 2: Signature now perfectly matches your .cpp file!
    SemanticEvents processEvents(const GlobalDataBank& robotData);

    // Getters for internal metrics to populate the Global Data Bus
    // float getDizzyBarYaw() const { return dizzyBarYaw; }
    // float getDizzyBarPitch() const { return dizzyBarPitch; }
    // float getDizzyBarRoll() const { return dizzyBarRoll; }
    // float getSmoothedTotalEnergy() const { return smoothedTotalEnergy; }
    // float getFrustrationLevel() const { return frustrationLevel; }
    
    // bool getIsDriving() const {return isDriving; }
    // bool getIsHandTeasing() const { return isHandTeasing; }
    // bool getIsHandVanishing() const { return isHandVanishing; }
    // bool getIsHandling() const { return isHandling; }
    // bool getHasExperiencedLift() const { return hasExperiencedLift; }
    // bool getIsLowering() const { return isLowering; }
    // bool getHasLanded() const { return hasLanded; }
    // bool getIsDizzy() const { return isDizzy; }
};