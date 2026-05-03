#pragma once

// ==========================================
// MISTER MISCHIEF V1 - MOTOR PWM CONFIGURATION
// ==========================================

// --- PWM Settings (Tamiya 3V Motor Safety Limit) ---
constexpr int PWM_FREQ = 5000;
constexpr int PWM_RESOLUTION = 8;
constexpr int PWM_MAX_DUTY_CYCLE = 90; // 35% of 255 (max) to simulate a safe 3.0V for the Tamiya Motors from 8.4V source