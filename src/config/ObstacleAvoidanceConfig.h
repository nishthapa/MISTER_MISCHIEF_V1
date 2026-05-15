#pragma once

namespace ObstacleConfig {
    // ==========================================
    // RADAR SWEEP & ESCAPE SEQUENCE TUNING
    // ==========================================

    // RANGE: 30.0f TO 100.0f // The baseline motor speed used during the escape maneuvers
    constexpr float BASE_SPEED = 40.0f;

    // RANGE: 500 TO 2000 // How long he blindly backs up after hitting a wall
    constexpr unsigned long BACKUP_DURATION_MS = 1000;

    // RANGE: 90.0f TO 360.0f // How far the robot physically turns while scanning
    constexpr float SWEEP_ANGLE_DEG = 160.0f;

    // RANGE: 2000 TO 5000 // Max time allowed to spin tracks in the air during radar sweep
    constexpr unsigned long SWEEP_TIMEOUT_MS = 3000;

    // RANGE: 2000 TO 6000 // Max time allowed to try aligning to the escape heading (Infinite Loop Fix)
    constexpr unsigned long ALIGN_TIMEOUT_MS = 4000;

    // RANGE: 2.0f TO 10.0f // How accurate the alignment must be before driving forward
    constexpr float ALIGN_SUCCESS_TOLERANCE_DEG = 5.0f;

    // RANGE: 500 TO 2000 // How long he drives forward to escape the obstacle
    constexpr unsigned long ESCAPE_DURATION_MS = 1000;
}