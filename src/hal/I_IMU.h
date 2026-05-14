#pragma once

// A clean data structure containing fused, drift-free angles
struct FusedAngles {
    float yaw;             // Z-axis rotation (Relative Heading - Drifts)
    float pitch;           // Y-axis rotation (Nose up/down)
    float roll;            // X-axis rotation (Tilting left/right)
    
    // --- THE FUTURE-PROOF COMPASS DATA ---
    bool hasCompass;       // Tells the robot if it can trust the absolute heading
    float compassHeading;  // Absolute Magnetic North (0 to 360 degrees)
};

// The Contract: Every IMU must implement these functions
class I_IMU {
public:
    virtual bool init() = 0;
    virtual FusedAngles getAngles() = 0;

    // === THE CLI CALIBRATION API ===
    virtual void calibrateGyro() = 0;
    virtual void calibrateAccel() = 0;
    virtual void calibrateMag() = 0; // Magnetometer (Compass)
    
    // Virtual destructor for safe cleanup
    virtual ~I_IMU() {} 
};