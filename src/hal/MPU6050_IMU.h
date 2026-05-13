#pragma once
#include <Arduino.h>

// A clean data structure containing fused, drift-free angles
struct FusedAngles {
    float yaw;   // Z-axis rotation (Heading)
    float pitch; // Y-axis rotation (Nose up/down)
    float roll;  // X-axis rotation (Tilting left/right)
};

class MPU6050_IMU {
private:
    int sdaPin;
    int sclPin;
    int intPin;         // Changed from enPin to intPin
    uint8_t deviceAddr; // Now populated dynamically

    FusedAngles lastKnownAngles = {0, 0, 0}; // Memory for the last valid reading

public:
    // Dependency Injection: Pass the blueprint pins through the constructor
    // Constructor now takes the EN pin and the registered I2C Address
    MPU6050_IMU(int sda, int scl, int interruptPin, uint8_t address);

    // Boots the I2C bus and injects the proprietary firmware blob into the DMP of the MPU6050
    bool init();

    // Allows the RTOS task to check if the DMP has pushed a new physics packet
    bool isDataReady();

    // Pulls the raw packet, does the 3D math, and returns clean degrees
    FusedAngles getAngles();
};