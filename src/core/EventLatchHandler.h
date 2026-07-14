#pragma once
#include <Arduino.h>
#include "hal/interfaces/I_IMU.h"
#include "core/RobotState.h"
#include "core/GlobalDataBus.h" // <--- FIX 1: Now it knows what GlobalDataBank is!

// --- THE CORRECT ESPRESSIF TFLITE HEADERS ---
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
// No need for schema_generated.h or version.h unless you do strict version checking.
#include "AI_LatchSetterModel.h" // Your model file

// 1. SEMANTIC EVENTS (The Physical Truths + AI Latches)
struct SemanticEvents {
    bool TENSORFLOW_ALIVE = false;
    float smoothedTotalEnergy = 0.0f;
    float rawYawEnergy = 0.0f;
    float rawPitchEnergy = 0.0f;
    float rawRollEnergy = 0.0f;
    float totalRawEnergy = 0.0f;

    // --- DETERMINISTIC STATES (Calculated in C++) ---
    bool isAbsolutelyStill = false;
    
    // Orientations
    bool isUpright = true;
    bool isUpsideDown = false;
    bool isTippedLeft = false;
    bool isTippedRight = false;
    bool isNoseUp = false;
    bool isNoseDown = false;

    bool isStuck = false;           
    bool isBeingTeased = false;     
    bool isBeingPushed = false;

    // --- AI STATES (Calculated by the Neural Network) ---
    bool hasExperiencedLift = false;
    bool isHandling = false;
    bool isFreeFalling = false;
    bool isLowering = false;
    bool hasLanded = false;
    bool hazardDetected = false;
    bool isImpactDetected = false; // Added to match your model's 4th target
    
};

class EventLatchHandler {
private:
    // Only the memory strictly needed for the Pre-AI filter!
    float smoothedTotalEnergy;
    bool isHandling;

    float lastDistance;
    FusedAngles lastAngles;

    // --- AI PERCEPTION MEMORY & ARENA ---
    bool isAIInitialized = false;
    const tflite::Model* tflModel = nullptr;
    tflite::MicroInterpreter* interpreter = nullptr;
    TfLiteTensor* input = nullptr;
    TfLiteTensor* output = nullptr;

    // 8KB Tensor Arena (Espressif optimized)
    static constexpr int kTensorArenaSize = 8 * 1024;
    uint8_t tensor_arena[kTensorArenaSize] __attribute__((aligned(4)));

    void setupAI();
    
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