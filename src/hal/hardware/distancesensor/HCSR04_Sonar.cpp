#include "hal/hardware/distancesensor/HCSR04_Sonar.h"
#include "config/SensorConfig.h"
#include <Arduino.h>

// FreeRTOS Hardware Lock to prevent OS preemption
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// The Constructor maps your hardware pins to the internal variables
HCSR04_Sonar::HCSR04_Sonar(int trig, int echo) {
    trigPin = trig;
    echoPin = echo;
}

void HCSR04_Sonar::init() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    
    // Ensure the trigger is dead silent on boot
    digitalWrite(trigPin, LOW); 
}

float HCSR04_Sonar::getDistanceCM() {
    // 1. THE HARDWARE RESET (Fixes the HC-SR04 "Stuck Echo" Bug)
    // Physically pull the pin down to clear any locked internal timers from previous missed pings.
    pinMode(echoPin, OUTPUT);
    digitalWrite(echoPin, LOW);
    delayMicroseconds(2);
    pinMode(echoPin, INPUT);

    // 2. THE OS LOCK (Fixes FreeRTOS Preemption)
    // Lock the CPU scheduler so the RTOS cannot pause us during the critical 10us trigger.
    portENTER_CRITICAL(&mux);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    portEXIT_CRITICAL(&mux);

    // 3. Read the pulse
    unsigned long duration = pulseIn(echoPin, HIGH, DistanceSensorConfig::ECHO_TIMEOUT_US);

    float rawDistance;
    if (duration == 0) {
        rawDistance = 400.0f; // Clamp timeouts to Max Range
    } else {
        rawDistance = (duration * DistanceSensorConfig::SPEED_OF_SOUND_CM_US) / 2.0;
    }

    // ==========================================
    // 4. ASYMMETRIC OUTLIER REJECTION
    // ==========================================
    const float MAX_PHYSICAL_JUMP_CM = 15.0f; 
    float delta = rawDistance - lastAcceptedRaw;

    // If it's not the first boot, and the jump is physically impossible...
    if (lastAcceptedRaw > 0.0f && abs(delta) > MAX_PHYSICAL_JUMP_CM) {
        
        if (delta < 0.0f) {
            // RULE A: THE SUDDEN OBSTACLE (e.g. A hand appeared)
            // Distance dropped. Ricochets can't cause this. Accept immediately!
            lastAcceptedRaw = rawDistance;
            outlierStreak = 0;
        } else {
            // RULE B: THE RICOCHET (e.g. 56cm -> 250cm)
            // Distance spiked. Quarantine this reading for 4 frames (200ms).
            outlierStreak++;
            if (outlierStreak < 4) {
                rawDistance = lastAcceptedRaw; // Ignore the spike
            } else {
                lastAcceptedRaw = rawDistance; // Object actually moved away
                outlierStreak = 0;
            }
        }
        
    } else {
        // Valid physical movement. Accept and reset.
        lastAcceptedRaw = rawDistance;
        outlierStreak = 0;
    }

    // 5. Feed the physics-verified distance into the Median Filter
    float medianDistance = filter.addSample(rawDistance);

    // ==========================================
    // 6. ADAPTIVE LOW-PASS FILTER (Dynamic Bias)
    // ==========================================
    if (emaDistance < 0.0f) {
        emaDistance = medianDistance; // Initialize
    } else {
        float alpha = EMA_ALPHA; // Default to heavy smoothing for PID stability
        
        // If the new reading is radically different, it's a sudden physical event.
        // Increase the bias to 1.0 to snap to the new reality instantly!
        if (abs(medianDistance - emaDistance) > 10.0f) {
            alpha = 0.9f; 
        }

        emaDistance = (alpha * medianDistance) + ((1.0f - alpha) * emaDistance);
    }

    return emaDistance;
}