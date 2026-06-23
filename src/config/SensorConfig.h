#pragma once

#include <Arduino.h>

// ==========================================
// 1. SMART COMBO CATALOG (The "Board" Level)
// ==========================================
enum class ComboModule {
    NONE,
    MODULE_MPU6050,    // The old I2C sensor
    MODULE_GY91,       // The new SPI/I2C Combo module (MPU9250 + Mag + Baro)
    MODULE_GY91_CLONE  // The clone module (MPU6500 + BMP280, No Mag)
};

enum class BusType { I2C, SPI };

// ==========================================
// 2. ACTIVE HARDWARE SELECTION
// ==========================================
// Select your main IMU/Sensor board here:
constexpr ComboModule ACTIVE_MODULE = ComboModule::MODULE_GY91_CLONE;

// Select how the module is wired to the ESP32:
constexpr BusType ACTIVE_BUS = BusType::I2C; 

// ==========================================
// 3. DISTANCE SENSOR CONFIG
// ==========================================
namespace DistanceSensorConfig {
    // === THE MUSEUM CATALOG ===
    enum DistanceModel {
        SENSOR_NONE,
        SENSOR_HCSR04, // Ultrasonic
        SENSOR_VL53L0X // Laser Time-of-Flight (For later!)
    };

    // SELECT YOUR DISTANCE SENSOR HARDWARE
    constexpr DistanceModel SELECTED_DIST_SENSOR = SENSOR_HCSR04;

    // --- HC-SR04 (ULTRASONIC) Specific Tuning ---
    constexpr float SPEED_OF_SOUND_CM_US = 0.0343f;
    constexpr unsigned long ECHO_TIMEOUT_US = 25000;
    constexpr unsigned int TRIGGER_CLEAR_DELAY_US = 2;  
    constexpr unsigned int TRIGGER_PULSE_DELAY_US = 10; 
    constexpr unsigned int SONAR_MAX_DIST = 350;
}

// ==========================================
// 4. INERTIAL MEASUREMENT UNIT (IMU)
// ==========================================
namespace IMUConfig {
    // === THE MUSEUM CATALOG ===
    enum IMUModel {
        IMU_NONE,
        IMU_MPU6050,
        IMU_MPU6500,
        IMU_MPU9250, 
        IMU_BNO055 
    };

    // --- SMART COMBO RESOLUTION ---
    // Automatically deduces the correct internal IMU silicon based on the ACTIVE_MODULE
    constexpr IMUModel SELECTED_IMU = 
        (ACTIVE_MODULE == ComboModule::MODULE_GY91) ? IMU_MPU9250 :
        (ACTIVE_MODULE == ComboModule::MODULE_GY91_CLONE) ? IMU_MPU6500 :
        (ACTIVE_MODULE == ComboModule::MODULE_MPU6050) ? IMU_MPU6050 :
        IMU_NONE;

    // ==========================================
    // COMPASS SETTINGS
    // ==========================================
    constexpr bool HAS_COMPASS = (SELECTED_IMU == IMU_BNO055 || 
                                  SELECTED_IMU == IMU_MPU9250);

    // ==========================================
    // FOR INVENSENSE IMUS (MPU6050/6500/9250/9255)
    // ==========================================
    constexpr bool MPU6050_USE_HARDWARE_DMP = false;
    constexpr bool MPU6500_USE_HARDWARE_DMP = true;
    constexpr uint8_t MPU6050_I2C_READ_TIMEOUT_MS = 50;

    constexpr float ACCEL_SCALE_FACTOR = 16384.0f;
    constexpr float GYRO_SCALE_FACTOR = 131.0f;
    constexpr float GYRO_DEADBAND_RAD_S = 0.005f;

    constexpr uint8_t AUTO_CALIBRATION_SAMPLES = 6;
    constexpr int16_t DEFAULT_ZACCEL_OFFSET = 1688;
    constexpr int16_t GYRO_CALIBRATION_SAMPLES = 500; 
    constexpr int16_t ACCEL_CALIBRATION_SAMPLES = 500;

    constexpr float MADGWICK_BETA = 0.04f;

    // ==========================================
    // SENSOR HARDWARE TIMINGS
    // ==========================================
    // Synchronize this with the MAIN_LOOP_TICK_RATE_MS (10ms = 100Hz)
    // 100Hz provides incredibly smooth Madgwick tracking without wasting CPU.
    constexpr uint16_t IMU_DMP_SAMPLE_RATE_HZ = 100; // Must be equal to or faster than the main loop tick rate for optimal performance
    constexpr uint16_t IMU_RAW_SAMPLE_RATE_HZ = 200; // If DMP fails, we can still get good performance at 200Hz with the Madgwick filter (just more CPU load)
}

// ==========================================
// 5. BAROMETER CONFIG
// ==========================================
namespace BarometerConfig {
    // === THE MUSEUM CATALOG ===
    enum BarometerModel {
        BARO_NONE,
        BARO_BMP280
    };

    // --- SMART COMBO RESOLUTION ---
    // Automatically deduces if a barometer is present on the board
    constexpr BarometerModel SELECTED_BAROMETER = 
        (ACTIVE_MODULE == ComboModule::MODULE_GY91 || 
         ACTIVE_MODULE == ComboModule::MODULE_GY91_CLONE) 
         ? BARO_BMP280 : BARO_NONE;
}