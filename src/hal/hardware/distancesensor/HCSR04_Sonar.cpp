#include "hal/hardware/distancesensor/HCSR04_Sonar.h"
#include "config/SensorConfig.h"
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
    delayMicroseconds(DistanceSensorConfig::TRIGGER_CLEAR_DELAY_US);

    // 2. Fire the 10-microsecond ultrasonic pulse
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(DistanceSensorConfig::TRIGGER_PULSE_DELAY_US);
    digitalWrite(trigPin, LOW);

    // 3. Measure the echo pulse length
    // CRITICAL FREERTOS SAFETY: The "25000" is a timeout in microseconds.
    // Without this timeout, if a wire disconnects, pulseIn() would wait forever,
    // locking up Core 0. 25,000us max wait = roughly 400cm maximum range.
    unsigned long duration = pulseIn(echoPin, HIGH, DistanceSensorConfig::ECHO_TIMEOUT_US);

    // 4. Handle errors (Timeout or wiring issue)
    // Send the -1.0 to the filter anyway. If it's a 1-frame glitch, 
    // the median filter will delete it. If the wire is actually unplugged, 
    // the buffer will fill with -1.0s and correctly output the error!
    if (duration == 0) {
        return -1.0; 
    }

    // 5. Calculate raw distance from the sensor
    // The speed of sound is roughly 0.0343 cm per microsecond.
    // We divide by 2 because the sound wave travels to the wall AND back.
    float rawDistance = (duration * DistanceSensorConfig::SPEED_OF_SOUND_CM_US) / 2.0;
    
    // 6. Return the filtered reality
    return filter.addSample(rawDistance);
}