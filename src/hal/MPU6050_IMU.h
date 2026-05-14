#pragma once
#include <Arduino.h>
#include "hal/I_IMU.h" // <-- Include the master interface

// Notice we inherit from I_IMU here!
class MPU6050_IMU : public I_IMU {
private:
    int sdaPin;
    int sclPin;
    int intPin;         
    uint8_t deviceAddr; 

    FusedAngles lastKnownAngles; 

    // === ESP32 SOFTWARE FILTER MEMORY ===
    unsigned long lastUpdateTime;
    float compPitch = 0.0f;
    float compRoll  = 0.0f;
    float compYaw   = 0.0f;

public:
    MPU6050_IMU(int sda, int scl, int interruptPin, uint8_t address);

    // Fulfilling the I_IMU Contract
    bool init() override;
    FusedAngles getAngles() override;

    // MPU6050 specific functions
    bool isDataReady();
};