#include "hal/hardware/distancesensor/HCSR04_Sonar.h"
#include "config/SensorConfig.h"
#include <Arduino.h>

HCSR04_Sonar::HCSR04_Sonar(int trig, int echo) {
    trigPin = trig;
    echoPin = echo;
    sonar = nullptr; // Initialize the pointer to safe null
}

HCSR04_Sonar::~HCSR04_Sonar() {
    if (sonar != nullptr) {
        delete sonar;
    }
}

void HCSR04_Sonar::init() {
    // SAFELY allocate memory now that FreeRTOS is booted!
    sonar = new NewPing(trigPin, echoPin, (int)DistanceSensorConfig::SONAR_MAX_DIST);
    emaDistance = -1.0f;
}

float HCSR04_Sonar::getDistanceCM() {
    // 1. ACOUSTIC LOCKOUT CACHE (The 40ms Rule)
    // We still need this to prevent the physical sound waves from colliding in the room!
    static unsigned long lastPingTime = 0;
    if (lastPingTime != 0 && millis() - lastPingTime < 40) {
        return emaDistance;
    }
    lastPingTime = millis();

    // ==========================================
    // 2. THE NEWPING FIRE & READ
    // ==========================================
    // No CPU locks. No MUX. FreeRTOS can preempt this safely.
    // It returns 0 if the ping is out of range.
    unsigned long ping_us = sonar->ping(); 

    float rawDistance;
    if (ping_us == 0) {
        // Blindspot / Out of Range handler
        if (lastAcceptedRaw > 0.0f && lastAcceptedRaw < 25.0f) {
            blindspotStreak++;
            if (blindspotStreak < 15) rawDistance = 2.0f; 
            else rawDistance = DistanceSensorConfig::SONAR_MAX_DIST; 
        } else {
            rawDistance = DistanceSensorConfig::SONAR_MAX_DIST; 
            blindspotStreak = 0;
        }
    } else {
        // Convert to CM using NewPing's built-in macro for the speed of sound
        rawDistance = ping_us / (float)US_ROUNDTRIP_CM;
        blindspotStreak = 0;
    }

    // ==========================================
    // 3. PHYSICAL RICOCHET FILTER
    // ==========================================
    // The OS hallucinations are dead, but we still need to filter physical sound ricochets!
    const float MAX_PHYSICAL_JUMP_CM = 15.0f; 
    float delta = rawDistance - lastAcceptedRaw;

    if (lastAcceptedRaw > 0.0f && abs(delta) > MAX_PHYSICAL_JUMP_CM) {
        if (delta < 0.0f) {
            lastAcceptedRaw = rawDistance;
            outlierStreak = 0;
        } else {
            outlierStreak++;
            if (outlierStreak < 4) {
                rawDistance = lastAcceptedRaw; 
            } else { 
                lastAcceptedRaw = rawDistance; 
                outlierStreak = 0; 
            }
        }
    } else {
        lastAcceptedRaw = rawDistance;
        outlierStreak = 0;
    }

    // 4. Feed the clean data to the Median Filter to kill SPIKES
    float medianDistance = filter.addSample(rawDistance);
    
    // disabled EMA filter to avoid sonar phase lag
    // 5. Feed the Median result into the EMA Filter to kill FUZZ
    // if (emaDistance < 0.0f) {
    //     // If it's the very first boot ping, sync the cache to reality!
    //     emaDistance = medianDistance;
    // } else {
    //     // EMA Math: Blend the new reading with the historical average
    //     // (Assuming EMA_ALPHA is defined in your header, usually ~0.2f to 0.4f)
    //     emaDistance = (EMA_ALPHA * medianDistance) + ((1.0f - EMA_ALPHA) * emaDistance);
    // }

    //return emaDistance;
    return medianDistance;
}