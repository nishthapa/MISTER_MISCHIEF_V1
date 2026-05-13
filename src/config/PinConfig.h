#pragma once

// ==========================================
// MISTER MISCHIEF V1 - PIN CONFIGURATION
// ==========================================

// --- Motor Driver Pins (XY-160D) ---
// Note: Wired to the unified 6-pin left-side header
namespace HardwarePins {
    constexpr int PIN_MOTOR_LEFT_FWD  = 4;
    constexpr int PIN_MOTOR_LEFT_REV  = 5;
    constexpr int PIN_MOTOR_RIGHT_FWD = 6;
    constexpr int PIN_MOTOR_RIGHT_REV = 10;


    // --- MPU6050 IMU Pins (I2C) ---
    constexpr int PIN_I2C_SCL = 9; // correct
    constexpr int PIN_I2C_SDA = 8; // correct
    constexpr int PIN_IMU_INT = 7; // correct //The hardware Interrupt pin

    // --- Sensor Pins (HC-SR04) ---
    // WARNING: ECHO pin must route through the 1k/2k voltage divider!
    constexpr int PIN_SONAR_TRIG = 15;
    constexpr int PIN_SONAR_ECHO = 16;
}