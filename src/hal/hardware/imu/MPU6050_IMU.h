#pragma once
#include <Arduino.h>
#include "hal/interfaces/I_IMU.h" 
#include "core/MadgwickFilter.h" 

class MPU6050_IMU : public I_IMU {
private:
    int sdaPin, sclPin, intPin;         
    uint8_t deviceAddr; 

    FusedAngles lastKnownAngles; 
    MadgwickFilter* filter; 

    unsigned long lastUpdateTime;

    // === BACKGROUND GYRO CALIBRATION MEMORY ===
    bool calibratingGyro = false;
    int calibrationSamples = 0;
    long sumX = 0, sumY = 0, sumZ = 0;
    float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

    // === BACKGROUND ACCEL CALIBRATION MEMORY ===
    bool calibratingAccel = false;
    int accelCalibSamples = 0;
    long sumAccelX = 0, sumAccelY = 0, sumAccelZ = 0;
    float accelBiasX = 0.0f, accelBiasY = 0.0f, accelBiasZ = 0.0f;

public:
    MPU6050_IMU(int sda, int scl, int interruptPin, uint8_t address);

    bool init() override;

    void setFilterBeta(float beta) override { 
        if (filter) filter->updateBeta(beta); 
    }
    
    FusedAngles getAngles() override;
    
    // Satisfy the interface
    void calibrateGyro() override;
    void calibrateAccel() override;
    void calibrateMag() override;
};