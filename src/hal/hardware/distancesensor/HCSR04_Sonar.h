#pragma once

#include "hal/interfaces/I_DistanceSensor.h"
#include "utils/MedianFilter.h"

class HCSR04_Sonar : public I_DistanceSensor {
private:
    int trigPin;
    int echoPin;

    // Instantiate a pluggable filter with a window size of 5
    MedianFilter<5> filter;

    // --- PHYSICS CONSTRAINTS ---
    float lastAcceptedRaw = 0.0f;
    int outlierStreak = 0;

public:
    // Constructor: Forces the main program to assign the pins
    HCSR04_Sonar(int trig, int echo);

    // Configures the pin modes
    void init() override;

    // Fires the acoustic pulse and calculates the distance in centimeters.
    // Includes a hard timeout to prevent FreeRTOS task freezing.
    // Returns -1.0 if it reads nothing (out of range or disconnected wire).
    float getDistanceCM() override;
};