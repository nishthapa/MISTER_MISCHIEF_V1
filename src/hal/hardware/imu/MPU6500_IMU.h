#pragma once

#include <Arduino.h>
#include "hal/interfaces/I_IMU.h" 
#include "core/MadgwickFilter.h" 
#include "utils/MedianFilter.h"
#include <MPU9250-DMP.h>

class MPU6500_IMU : public I_IMU {
private:
    int sdaPin, sclPin; 
    int ncsPin; // Used to detect SPI vs I2C
    uint8_t deviceAddr; 
    bool useSPI;

    MPU9250_DMP imu;
    FusedAngles lastKnownAngles;

    // EMI filters for the raw bypass mode
    MedianFilter<5> filterAx, filterAy, filterAz;
    MedianFilter<5> filterGx, filterGy, filterGz;
    MadgwickFilter* filter;

    unsigned long lastUpdateTime;
    bool usingDMP;

public:
    // Unified Constructor: Pass -1 for ncs to force I2C mode
    MPU6500_IMU(int sda, int scl, int ncs, uint8_t address);

    bool init() override;
    FusedAngles getAngles() override;
    float getTemperature() override;
    
    void setFilterBeta(float beta) override { 
        if (filter) filter->updateBeta(beta); 
    }
    
    void calibrateGyro() override;
    void calibrateAccel() override;
    void calibrateMag() override;
};