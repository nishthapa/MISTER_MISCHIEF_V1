#pragma once

#include "hal/interfaces/I_DistanceSensor.h"
#include "utils/MedianFilter.h"
#include <NewPing.h> // using tecle12's NewPing sonar library

class HCSR04_Sonar : public I_DistanceSensor {
private:
// We brought these back!
    int trigPin;
    int echoPin;
    NewPing* sonar; 
    MedianFilter<5> filter;

    float lastAcceptedRaw = 0.0f;
    int outlierStreak = 0;
    int blindspotStreak = 0; 

    float emaDistance = -1.0f;

    const float EMA_ALPHA = 0.4f;

public:
    HCSR04_Sonar(int trig, int echo);
    ~HCSR04_Sonar(); // We need a destructor to clean up the pointer!

    void init() override;
    float getDistanceCM() override;
};