#pragma once

namespace MotorDriverConfig {
    // === THE MUSEUM CATALOG ===
    enum MotorDriverModel {
        DRIVER_XY160D,
        DRIVER_L298N, // For future upgrades!
        DRIVER_TB6612 // For future upgrades!
    };

    // 1. SELECT YOUR HARDWARE
    constexpr MotorDriverModel SELECTED_DRIVER = DRIVER_XY160D;

    // ==========================================
    // 2. HARDWARE LIMITS & FREQUENCIES
    // ==========================================
    // RANGE: 1000 TO 20000 // PWM frequency in Hz. High frequency = silent, but less low-end torque.
    constexpr int PWM_FREQ = 10000;
    
    // RANGE: 8 TO 12 // Resolution. 8-bit means speeds are 0-255. 10-bit means 0-1023.
    constexpr int PWM_RES = 10; // 10-bit resolution gives us finer control at low speeds, but may cause more EMI noise.
    
    constexpr int MAX_DUTY = 1023;
    //constexpr int MIN_DUTY = -1023;
    constexpr int MIN_DUTY = 0;

    // ==========================================
    // 3. PHYSICAL FRICTION DEAD-BAND
    // ==========================================
    // RANGE: 0.0f TO 30.0f // The minimum requested speed required to overcome the gearbox friction. 
    //constexpr float DRIVER_DEADBAND = 10.0f; // Moved to our Dynamic ConfigurationManager and FactoryDefaults
}