#include "hal/HCSR04_Sonar.h"
#include <Arduino.h>

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
    // 1. Clear the trigger pin to ensure a clean high pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // 2. Fire the 10-microsecond ultrasonic pulse
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // 3. Measure the echo pulse length
    // CRITICAL FREERTOS SAFETY: The "25000" is a timeout in microseconds.
    // Without this timeout, if a wire disconnects, pulseIn() would wait forever,
    // locking up Core 0. 25,000us max wait = roughly 400cm maximum range.
    unsigned long duration = pulseIn(echoPin, HIGH, 25000);

    // 4. Handle errors (Timeout or wiring issue)
    if (duration == 0) {
        return -1.0; 
    }

    // 5. Calculate distance
    // The speed of sound is roughly 0.0343 cm per microsecond.
    // We divide by 2 because the sound wave travels to the wall AND back.
    float distance = (duration * 0.0343) / 2.0;
    
    return distance;
}