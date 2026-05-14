#pragma once

namespace IMUConfig {
    // === THE MUSEUM CATALOG ===
    enum IMUModel {
        IMU_NONE,
        IMU_MPU6050,
        IMU_BNO055, // For later! // Has Compass
        IMU_MPU9250 // For later! // Has Compass
    };

    // 1. SELECT YOUR HARDWARE
    constexpr IMUModel SELECTED_IMU = IMU_MPU6050;

    // ==========================================
    // 2. MPU6050 SPECIFIC SETTINGS
    // ==========================================
    // true = Hardware DMP (Firmware upload, 100kHz I2C limit, requires shielded wires)
    // false = Raw Data + ESP32 Software Filter (10ms Boot, 400kHz I2C, Bulletproof)
    constexpr bool MPU6050_USE_HARDWARE_DMP = false;

    // ==========================================
    // 3. COMPASS SETTINGS
    // ==========================================
    // If you select an IMU with a compass, this automatically becomes true!
    constexpr bool HAS_COMPASS = (SELECTED_IMU == IMU_BNO055 || SELECTED_IMU == IMU_MPU9250);

    // ==========================================
    // 4. SOFTWARE FILTER TUNING (Raw Data Mode)
    // ==========================================
    // 0.98 means: Trust the Gyroscope 98% of the time (smooth but drifts),
    // and trust the Accelerometer 2% of the time (noisy but knows exactly where gravity is).
    //constexpr float COMP_FILTER_ALPHA = 0.98f;
    constexpr float MADGWICK_BETA = 0.04f; // Lowered to kill the twitching!
}