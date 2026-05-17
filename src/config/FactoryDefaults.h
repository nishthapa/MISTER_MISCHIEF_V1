#pragma once

namespace FactoryDefaults {
    // --- Personality Settings ---
    constexpr float CRUISING_SPEED = 30.0f;
    constexpr float OBSTACLE_TRIGGER_CM = 30.0f;
    constexpr float MAINTAIN_DIST_CM = 15.0f;

    // --- System States ---
    constexpr bool BRAIN_ACTIVE = true;

    // --- Debug & Telemetry ---
    constexpr bool SERIAL_DEBUG_MASTER = true;
    constexpr bool SERIAL_DEBUG_IMU = true;
    constexpr bool SERIAL_DEBUG_SONAR = true;
    constexpr bool SERIAL_DEBUG_MOTOR_DRIVER = true;
}