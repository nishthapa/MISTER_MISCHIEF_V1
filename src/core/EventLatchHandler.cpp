#include "core/EventLatchHandler.h"
#include "config/ConfigurationManager.h"
#include "config/SystemConfig.h"
#include <cmath> 

EventLatchHandler::EventLatchHandler() { 
    smoothedTotalEnergy = 0.0f;
    isHandling = false;
    lastDistance = -1.0f;
    lastAngles = {0, 0, 0, 0, false, 0};
}

void EventLatchHandler::setupAI() {
    // Read the model from the byte array inside AI_LatchSetterModel.h
    tflModel = tflite::GetModel(mister_mischief_model);
    
    static tflite::AllOpsResolver resolver;
    static tflite::MicroInterpreter static_interpreter(
        tflModel, resolver, tensor_arena, kTensorArenaSize);
    
    interpreter = &static_interpreter;
    
    if (interpreter->AllocateTensors() != kTfLiteOk) {
        return; // Memory allocation error
    }
    
    input = interpreter->input(0);
    output = interpreter->output(0);
    isAIInitialized = true;
    //CurrentRobotData.health.hardwareBitmask |= Comms::HealthBit::TENSORFLOW_ALIVE;
}

SemanticEvents EventLatchHandler::processEvents(const GlobalDataBank& robotData) {
    if (!isAIInitialized) {
        setupAI();
    }

    SemanticEvents events; 

    // 1. CALCULATE DELTAS & ENERGY
    float currentDistance = (robotData.health.hardwareBitmask & Comms::HealthBit::SONAR_OK) ? robotData.sensors.distanceCM : -1.0f;
    float distanceDelta = (currentDistance != lastDistance && lastDistance > 0.0f) ? (currentDistance - lastDistance) : 0.0f;
    lastDistance = currentDistance;

    //float rawYawEnergy = 0.0f, rawPitchEnergy = 0.0f, rawRollEnergy = 0.0f;
    float currentYawRate = 0.0f; 
    
    if (robotData.health.hardwareBitmask & Comms::HealthBit::IMU_OK) {
        auto getShortestAngleDelta = [](float current, float previous) {
            float delta = current - previous;
            if (delta > 180.0f) delta -= 360.0f; else if (delta < -180.0f) delta += 360.0f;
            return delta;
        };

        float yawDelta = getShortestAngleDelta(robotData.physics.imuAngles.yaw, lastAngles.yaw);
        float pitchDelta = getShortestAngleDelta(robotData.physics.imuAngles.pitch, lastAngles.pitch);
        float rollDelta = getShortestAngleDelta(robotData.physics.imuAngles.roll, lastAngles.roll);
        events.rawYawEnergy = std::abs(yawDelta) / 0.01f;
        events.rawPitchEnergy = std::abs(pitchDelta) / 0.01f;
        events.rawRollEnergy = std::abs(rollDelta) / 0.01f;
        
        // Convert to true degrees/sec for the NN input tensor
        float dt = SystemConfig::MAIN_LOOP_TICK_RATE_MS / 1000.0f;
        currentYawRate = yawDelta / dt;

        lastAngles.yaw = robotData.physics.imuAngles.yaw;
        lastAngles.pitch = robotData.physics.imuAngles.pitch;
        lastAngles.roll = robotData.physics.imuAngles.roll;
    }

    // float totalRawEnergy = rawYawEnergy + rawPitchEnergy + rawRollEnergy;
    events.totalRawEnergy = events.rawYawEnergy + events.rawPitchEnergy + events.rawRollEnergy;
    smoothedTotalEnergy = (SysConfig.ENERGY_EMA_ALPHA * events.totalRawEnergy) + (SysConfig.ENERGY_EMA_BETA * smoothedTotalEnergy);
    events.smoothedTotalEnergy = smoothedTotalEnergy;

    // 2. STATIC ORIENTATIONS
    float pitch = robotData.physics.imuAngles.pitch;
    float roll = robotData.physics.imuAngles.roll;
    events.isUpright = false; 

    if (std::abs(roll) > 135.0f || std::abs(pitch) > 135.0f) {events.isUpsideDown = true; events.isUpright = false;}
    else if (pitch > 70.0f) {events.isNoseUp = true; events.isUpright = false;}
    else if (pitch < -70.0f) {events.isNoseDown = true; events.isUpright = false;}
    else if (roll > 70.0f) {events.isTippedRight = true; events.isUpright = false;}
    else if (roll < -70.0f) {events.isTippedLeft = true; events.isUpright = false;}

    else events.isUpright = true; // If it isn't any of the extreme edge cases, it's upright!

    // isAbsolutelyStill should be independent of isHandling
    // Break stillness INSTANTLY on a raw energy spike, bypassing the EMA lag
    events.isAbsolutelyStill = (!robotData.actuators.isDriving &&
                                smoothedTotalEnergy < SysConfig.SMOOTHED_PERFECTLY_STILL_ENERGY_MAX &&
                                events.totalRawEnergy < SysConfig.RAW_PERFECTLY_STILL_ENERGY_MAX);

    // ==========================================================
    // THE "LIFT LATCH"
    // (2-Stage Arming Logic for Human Pickup Detection)
    // ==========================================================
    static bool liftLatch = false;
    static bool energySpikeMemory = false;
    static uint16_t liftDebounceCounter = 0; // Tracks consecutive out-of-bounds ticks
    
    // We want 250ms of continuous violation to confirm a human lift.
    // 250ms / SysConfig.MAIN_LOOP_TICK_RATE_MS (10ms) = 25 ticks.
    const uint16_t LIFT_DEBOUNCE_TICKS = SysConfig.LIFT_UP_DETECTION_DELAY / SystemConfig::MAIN_LOOP_TICK_RATE_MS;
    
    // Normal driving stays near 1.0G but track impacts cause high-frequency spikes.
    // A human picking the robot up causes a SUSTAINED Z-axis acceleration or freefall.
    // if (robotData.physics.imuAngles.gForce < 0.7f || robotData.physics.imuAngles.gForce > 1.4f) {
    // Check ONLY the G-Force limits to increment the counter
    if (robotData.physics.imuAngles.gForce < SysConfig.GFORCE_LIFT_DOWN_THRESHOLD ||
        robotData.physics.imuAngles.gForce > SysConfig.GFORCE_LIFT_UP_THRESHOLD) {

        if (liftDebounceCounter < LIFT_DEBOUNCE_TICKS) liftDebounceCounter++;

        // Arm the flag if the energy spikes (The Initial Yank)
        if (events.totalRawEnergy > SysConfig.LIFT_ENERGY_SPIKE_THRESHOLD) {
            energySpikeMemory = true;
        }
    } else {
        // A human lift crosses 1.0G mid-air. Do not wipe the memory!
        // Instead, let the counter decay gently so a true return to rest safely disarms it.
        if (liftDebounceCounter > 0) {
            liftDebounceCounter--;
        }
    }
   
    // If the vibration lasts longer than our 250ms threshold,
    // & Energy has spiked at some point, it's a real lift!
    // So Trigger the latch only if BOTH conditions are met (The Hold)
    if (energySpikeMemory && liftDebounceCounter >= LIFT_DEBOUNCE_TICKS) {
        liftLatch = true;
    }
    
    // Safely reset everything when placed flat on the floor
    if (events.isAbsolutelyStill && events.isUpright) {
        liftLatch = false;
        energySpikeMemory = false; // Clear memory for the next lift detection arming
        liftDebounceCounter = 0; // Clear the counter for the next lift
    }
    events.hasExperiencedLift = liftLatch;

    // ==========================================================
    // 3. NEURAL NETWORK LAYER (TensorFlow Lite Micro)
    // ==========================================================
    if (isAIInitialized && interpreter && SystemConfig::USE_AI_LATCH_HANDLER) {
        events.TENSORFLOW_ALIVE = true;
        
        // Map features exactly to the training index order
        input->data.f[0] = pitch;
        input->data.f[1] = roll;
        input->data.f[2] = currentYawRate;
        input->data.f[3] = currentDistance;
        input->data.f[4] = robotData.sensors.pressureDeltaPa;
        input->data.f[5] = robotData.actuators.isDriving ? 1.0f : 0.0f;  
        input->data.f[6] = robotData.physics.imuAngles.gForce;
        input->data.f[7] = smoothedTotalEnergy;

        // Compute Inference
        interpreter->Invoke();

        // Sigmoid thresholds conversion (> 0.5f means active)
        events.isHandling       = output->data.f[0] > 0.5f;
        events.isFreeFalling    = output->data.f[1] > 0.5f;
        events.hazardDetected   = output->data.f[2] > 0.5f;
        events.isImpactDetected = output->data.f[3] > 0.5f;

    } else {
        events.TENSORFLOW_ALIVE = false;
        // Safe deterministic fallback filter if ML is turned off
        events.isFreeFalling = (robotData.physics.imuAngles.gForce < SysConfig.GFORCE_FREEFALL_THRESHOLD);
        if (!robotData.actuators.isDriving && events.totalRawEnergy > SysConfig.STEADY_HOLD_ENERGY_MAX) {
            isHandling = true; 
        } else if (smoothedTotalEnergy < SysConfig.SMOOTHED_PERFECTLY_STILL_ENERGY_MAX) {
            isHandling = false; 
        }
        events.isHandling = isHandling;
    }

    // --- THE SAFETY OVERRIDE ---
    // If the physical lift latch is active, force the handling state true 
    // regardless of what the AI thinks while motors are spinning down.
    if (liftLatch) {
        events.isHandling = true;
    }

    return events;
}