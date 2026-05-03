#pragma once

// ==========================================
// MISTER MISCHIEF V1 - ESP32 PWM CHANNEL REGISTRY
// ==========================================
// WARNING: The ESP32-S3 has exactly 8 hardware PWM channels (0-7).
// Claim your channels here to permanently prevent hardware collisions.

// --- XY-160D Motor Driver Channels ---
constexpr int CH_MOTOR_LEFT_FWD  = 0;
constexpr int CH_MOTOR_LEFT_REV  = 1;
constexpr int CH_MOTOR_RIGHT_FWD = 2;
constexpr int CH_MOTOR_RIGHT_REV = 3;

// --- Future Expansion (Phase 2 placeholders) ---
// constexpr int CH_CAMERA_SERVO_PAN  = 4;
// constexpr int CH_CAMERA_SERVO_TILT = 5;
// constexpr int CH_SYSTEM_BUZZER     = 6;
// constexpr int CH_LASER_POINTER     = 7;