#pragma once
#include <Arduino.h>

// ==========================================
// MISTER MISCHIEF V1 - I2C ADDRESS REGISTRY
// ==========================================
// WARNING: No two devices on the I2C bus can share an address.
namespace I2CRegistry {
    
constexpr uint8_t I2C_ADDR_MPU6050 = 0x68;
constexpr uint8_t I2C_ADDR_MPU6500 = 0x68;

constexpr uint8_t I2C_MPU6050_REG_TEMP_OUT_H = 0x41; // Register address for temperature data

// --- Future Expansion (Phase 2) ---
// constexpr uint8_t I2C_ADDR_OLED_DISPLAY = 0x3C;
}