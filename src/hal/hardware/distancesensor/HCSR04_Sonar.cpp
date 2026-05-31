#include "hal/hardware/distancesensor/HCSR04_Sonar.h"
#include "config/SensorConfig.h"
#include <Arduino.h>

// FreeRTOS Hardware Lock to prevent OS preemption
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

HCSR04_Sonar::HCSR04_Sonar(int trig, int echo) {
    trigPin = trig;
    echoPin = echo;
}

void HCSR04_Sonar::init() {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW); 
}

float HCSR04_Sonar::getDistanceCM() {
    static unsigned long lastPingTime = 0;
    if (lastPingTime != 0 && millis() - lastPingTime < 40) {
        return emaDistance;
    }
    lastPingTime = millis();

    // 1. THE HARDWARE RESET
    pinMode(echoPin, OUTPUT);
    digitalWrite(echoPin, LOW);
    delayMicroseconds(2);
    pinMode(echoPin, INPUT);

    unsigned long duration = 0;

    // ==========================================
    // THE RTOS ARMOR FIX
    // ==========================================
    // Lock the CPU. FreeRTOS cannot pause us while we do this math!
    portENTER_CRITICAL(&mux);
    
    // Fire the trigger
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Read the pulse *while* the CPU is locked! 
    // FreeRTOS is paused, so pulseIn will measure the exact physical microsecond perfectly.
    duration = pulseIn(echoPin, HIGH, DistanceSensorConfig::ECHO_TIMEOUT_US);
    
    // Release the CPU back to FreeRTOS
    portEXIT_CRITICAL(&mux);
    // ==========================================

    float rawDistance;
    if (duration == 0) {
        if (lastAcceptedRaw > 0.0f && lastAcceptedRaw < 25.0f) {
            blindspotStreak++;
            if (blindspotStreak < 15) rawDistance = 2.0f; 
            else rawDistance = 400.0f; 
        } else {
            rawDistance = 400.0f; 
            blindspotStreak = 0;
        }
    } else {
        rawDistance = (duration * DistanceSensorConfig::SPEED_OF_SOUND_CM_US) / 2.0;
        blindspotStreak = 0;
    }

    const float MAX_PHYSICAL_JUMP_CM = 15.0f; 
    float delta = rawDistance - lastAcceptedRaw;

    if (lastAcceptedRaw > 0.0f && abs(delta) > MAX_PHYSICAL_JUMP_CM) {
        if (delta < 0.0f) {
            lastAcceptedRaw = rawDistance;
            outlierStreak = 0;
        } else {
            outlierStreak++;
            if (outlierStreak < 4) rawDistance = lastAcceptedRaw; 
            else { lastAcceptedRaw = rawDistance; outlierStreak = 0; }
        }
    } else {
        lastAcceptedRaw = rawDistance;
        outlierStreak = 0;
    }

    float medianDistance = filter.addSample(rawDistance);
    emaDistance = medianDistance; 
    return medianDistance; 
}