#pragma once

// Include configs so the preprocessor knows what macros are defined
#include "MotorDriverConfig.h"
#include "SensorConfig.h"

// ==========================================
// MISTER MISCHIEF V1 - MODULAR PIN CONFIG
// ==========================================
namespace HardwarePins {

    // ==========================================
    // Bus Architecture. DEFINED SEPARATELY FROM SENSORS!
    // ==========================================
    // --- I2C BUS ---
    constexpr int PIN_I2C_SCL = 9;
    constexpr int PIN_I2C_SDA = 8;

    // --- SPI BUS ---
    constexpr int PIN_SPI_SCK  = 9;
    constexpr int PIN_SPI_MOSI = 8;
    constexpr int PIN_SPI_MISO = 7; // Its INT pin on the MPU6050 in the current hw layout

    // ==========================================
    // Motor Driver Pins
    // ==========================================
    // --- XY160D ---
    // Note: Wired to the unified 6-pin left-side header
    constexpr int PIN_MOTOR_ENA       = 6;  // Enable Left Motor
    constexpr int PIN_MOTOR_LEFT_FWD  = 5;  // IN1
    constexpr int PIN_MOTOR_LEFT_REV  = 4;  // IN2

    // Note: Wired to the unified 6-pin right-side header
    constexpr int PIN_MOTOR_RIGHT_FWD = 42; // IN3
    constexpr int PIN_MOTOR_RIGHT_REV = 41; // IN4
    constexpr int PIN_MOTOR_ENB       = 40; // Enable Right Motor

    // ==========================================
    // IMU Specific Pins
    // ==========================================
    // --- MPU6050 ---
    constexpr int PIN_IMU_INT = 7; // Hardware Interrupt for MPU6050

    // --- MPU6500 ---
    constexpr int PIN_MPU_CS = 15; // Chip Select for BMP280 on SPI (NCS PIN on GY91 Combo module)
    //constexpr int PIN_BMP_CS = 7;  // Chip Select for BMP280 on SPI (GY91 Combo) (CSB PIN)

    // --- MPU9250 ---
    constexpr int PIN_MPU9250_CS = 10; // Chip Select for MPU9250 on SPI (GY91 Combo)

    // ==========================================
    // Barometer Specific Pins
    // ==========================================
    // --- BMP280 ---
    constexpr int PIN_BMP_CS = 11; // Chip Select for BMP280 on SPI (CSB PIN on GY91 Combo module)

    // ==========================================
    // Distance Sensor Pins
    // ==========================================
    // --- HCSR04 ---
    // WARNING: If its not a HCSR04P, ECHO pin must route through the 1k/2k voltage divider!
    constexpr int PIN_SONAR_TRIG = 39;
    constexpr int PIN_SONAR_ECHO = 36;
    
}