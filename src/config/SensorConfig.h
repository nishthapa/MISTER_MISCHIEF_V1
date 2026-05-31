#pragma once
#include <Arduino.h>

// ==========================================
// 1. DISTANCE SENSOR CONFIG
// ==========================================
namespace DistanceSensorConfig {
    // === THE MUSEUM CATALOG ===
    enum DistanceModel {
        SENSOR_NONE,
        SENSOR_HCSR04, // Ultrasonic
        SENSOR_VL53L0X // Laser Time-of-Flight (For later!)
    };

    // 1. SELECT YOUR SENSOR HARDWARE
    constexpr DistanceModel SELECTED_SENSOR = SENSOR_HCSR04;

    // --- HC-SR04 (ULTRASONIC) Specific Tuning ---
        // RANGE: 0.0330f TO 0.0350f // Speed of sound in air (cm per microsecond).
    // Note: 0.0343 is standard for sea level at 20°C. Cold/high altitude requires adjustment!
    constexpr float SPEED_OF_SOUND_CM_US = 0.0343f;

    // RANGE: 10000 TO 30000 // Microseconds to wait before giving up. 25,000us = ~400cm max range.
    constexpr unsigned long ECHO_TIMEOUT_US = 25000;
    
    // --- Hardware Pin Timings ---
    constexpr unsigned int TRIGGER_CLEAR_DELAY_US = 2;  // Time to stabilize the pin LOW
    constexpr unsigned int TRIGGER_PULSE_DELAY_US = 10; // Time to hold the pin HIGH to fire the pulse
    constexpr unsigned int SONAR_MAX_DIST = 350;
}

// ==========================================
// 2. INERTIAL MEASUREMENT UNIT (IMU)
// ==========================================
namespace IMUConfig {
    // === THE MUSEUM CATALOG ===
    enum IMUModel {
        IMU_NONE,
        IMU_MPU6050,
        IMU_BNO055, // For later! // Has Compass
        IMU_MPU9250 // For later! // Has Compass
    };

    // 1. SELECT YOUR IMU HARDWARE
    constexpr IMUModel SELECTED_IMU = IMU_MPU6050;

    // ==========================================
    // MPU6050 SPECIFIC SETTINGS
    // ==========================================
    // true = Hardware DMP (Firmware upload, 100kHz I2C limit, requires shielded wires)
    // false = Raw Data + ESP32 Software Filter (10ms Boot, 400kHz I2C, Bulletproof)
    constexpr bool MPU6050_USE_HARDWARE_DMP = false;

    // --- Physical Scale Factors ---
    // 16384.0 = +/- 2G | 8192.0 = +/- 4G | 4096.0 = +/- 8G | 2048.0 = +/- 16G
    constexpr float ACCEL_SCALE_FACTOR = 16384.0f;
    
    // 131.0 = +/- 250 deg/s | 65.5 = +/- 500 deg/s | 32.8 = +/- 1000 deg/s
    constexpr float GYRO_SCALE_FACTOR = 131.0f;

    // --- Noise Filtering ---
    // Ignore microscopic vibrations below this threshold (in radians/sec)
    constexpr float GYRO_DEADBAND_RAD_S = 0.005f;

    // --- Calibration ---
    constexpr uint8_t AUTO_CALIBRATION_SAMPLES = 6;
    constexpr int16_t DEFAULT_ZACCEL_OFFSET = 1688;
    constexpr int16_t GYRO_CALIBRATION_SAMPLES = 500; // Number of samples to take when calibrating gyro offsets
    constexpr int16_t ACCEL_CALIBRATION_SAMPLES = 500;

    // ==========================================
    // COMPASS SETTINGS
    // ==========================================
    // If you select an IMU with a compass, this automatically becomes true!
    constexpr bool HAS_COMPASS = (SELECTED_IMU == IMU_BNO055 || SELECTED_IMU == IMU_MPU9250);

    // ==========================================
    // 4. SOFTWARE FILTER TUNING (Raw Data Mode)
    // ==========================================
    // To-do: Move to Configurable Parameters in EEPROM for real-time tuning without code changes!
    constexpr float MADGWICK_BETA = 0.04f; // governs how much to trust the accelerometer // Lowered to kill the twitching!
}