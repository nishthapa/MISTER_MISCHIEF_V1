#pragma once

class HCSR04_Sonar {
private:
    int trigPin;
    int echoPin;

public:
    // Constructor: Forces the main program to assign the pins
    HCSR04_Sonar(int trig, int echo);

    // Configures the pin modes
    void init();

    // Fires the acoustic pulse and calculates the distance in centimeters.
    // Includes a hard timeout to prevent FreeRTOS task freezing.
    // Returns -1.0 if it reads nothing (out of range or disconnected wire).
    float getDistanceCM();
};