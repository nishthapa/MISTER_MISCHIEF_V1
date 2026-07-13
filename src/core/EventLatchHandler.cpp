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

    float rawYawEnergy = 0.0f, rawPitchEnergy = 0.0f, rawRollEnergy = 0.0f;
    float currentYawRate = 0.0f; 
    
    if (robotData.health.hardwareBitmask & Comms::HealthBit::IMU_OK) {
        auto getShortestAngleDelta = [](float current, float previous) {
            float delta = current - previous;
            if (delta > 180.0f) delta -= 360.0f; else if (delta < -180.0f) delta += 360.0f;
            return delta;
        };

        float yawDelta = getShortestAngleDelta(robotData.physics.imuAngles.yaw, lastAngles.yaw);
        rawYawEnergy = std::abs(yawDelta) / 0.01f;
        rawPitchEnergy = std::abs(getShortestAngleDelta(robotData.physics.imuAngles.pitch, lastAngles.pitch)) / 0.01f;
        rawRollEnergy = std::abs(getShortestAngleDelta(robotData.physics.imuAngles.roll, lastAngles.roll)) / 0.01f;
        
        // Convert to true degrees/sec for the NN input tensor
        float dt = SystemConfig::MAIN_LOOP_TICK_RATE_MS / 1000.0f;
        currentYawRate = yawDelta / dt;

        lastAngles.yaw = robotData.physics.imuAngles.yaw;
        lastAngles.pitch = robotData.physics.imuAngles.pitch;
        lastAngles.roll = robotData.physics.imuAngles.roll;
    }

    float totalRawEnergy = rawYawEnergy + rawPitchEnergy + rawRollEnergy;
    smoothedTotalEnergy = (SysConfig.ENERGY_EMA_ALPHA * totalRawEnergy) + (SysConfig.ENERGY_EMA_BETA * smoothedTotalEnergy);
    events.smoothedTotalEnergy = smoothedTotalEnergy;

    // 2. STATIC ORIENTATIONS
    float pitch = robotData.physics.imuAngles.pitch;
    float roll = robotData.physics.imuAngles.roll;
    events.isUpright = false; 

    if (std::abs(roll) > 135.0f || std::abs(pitch) > 135.0f) events.isUpsideDown = true;
    else if (pitch > 70.0f) events.isNoseUp = true;
    else if (pitch < -70.0f) events.isNoseDown = true;
    else if (roll > 70.0f) events.isTippedRight = true;
    else if (roll < -70.0f) events.isTippedLeft = true;
    // else if ((std::abs(pitch) < 30.0f && std::abs(roll) < 30.0f) ||
    //         (!events.isUpsideDown || !events.isNoseUp || !events.isNoseDown || !events.isTippedRight || !events.isTippedLeft)) events.isUpright = true;

    else events.isUpright = true; // If it isn't any of the extreme edge cases, it's upright!

    events.isAbsolutelyStill = (!robotData.actuators.isDriving && !events.isHandling && smoothedTotalEnergy < SysConfig.PERFECTLY_STILL_ENERGY);

    // ==========================================================
    // THE "LIFT LATCH" (Human Pickup Detection)
    // ==========================================================
    static bool liftLatch = false;
    
    // Normal driving stays near 1.0G. A human picking the robot up 
    // causes a sudden Z-axis acceleration or temporary freefall.
    if (robotData.physics.imuAngles.gForce < 0.65f || robotData.physics.imuAngles.gForce > 1.4f) {
        liftLatch = true;
    }
    
    // Reset the latch ONLY when the robot is placed safely back on the ground
    if (events.isAbsolutelyStill && events.isUpright) {
        liftLatch = false;
    }
    events.hasExperiencedLift = liftLatch;

    // ==========================================================
    // 3. NEURAL NETWORK LAYER (TensorFlow Lite Micro)
    // ==========================================================
    if (isAIInitialized && interpreter && SystemConfig::USE_AI_BEHAVIOUR_ENGINE) {
        
        // Map features exactly to the training index order
        input->data.f[0] = pitch;
        input->data.f[1] = roll;
        input->data.f[2] = currentYawRate;
        input->data.f[3] = currentDistance;
        input->data.f[4] = robotData.sensors.pressureDeltaPa;
        input->data.f[5] = robotData.actuators.isDriving ? 1.0f : 0.0f;

        // --- THE AI FIX ---
        // We force the network to ignore the motor states when predicting Handling/Impacts.
        // It must judge the physical IMU energy purely on its own merits.
        // input->data.f[5] = 0.0f; // Force isDriving to false for the AI
        
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
        // Safe deterministic fallback filter if ML is turned off
        events.isFreeFalling = (robotData.physics.imuAngles.gForce < 0.25f);
        if (!robotData.actuators.isDriving && totalRawEnergy > SysConfig.STEADY_HOLD_ENERGY_MAX) {
            isHandling = true; 
        } else if (smoothedTotalEnergy < SysConfig.PERFECTLY_STILL_ENERGY) {
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