#pragma once

namespace FactoryDefaults {
    // --- Personality Settings ---
    constexpr float CRUISING_SPEED = 30.0f;
    constexpr float OBSTACLE_TRIGGER_CM = 30.0f;
    constexpr float MAINTAIN_DIST_CM = 15.0f;

    // --- System States ---
    constexpr bool BRAIN_ACTIVE = true;

    // --- Network & Comms ---
    constexpr const char* WIFI_SSID = "";       // Blank by default!
    constexpr const char* WIFI_PASSWORD = "";
    constexpr bool WIFI_ACTIVE = false;         // Off by default to save battery
    
    constexpr const char* BT_NAME = "Mister_Mischief";
    constexpr bool BT_ACTIVE = false;

    // --- Debug & Telemetry ---
    constexpr bool SERIAL_DEBUG_MASTER = true;
    constexpr bool SERIAL_DEBUG_IMU = true;
    constexpr bool SERIAL_DEBUG_SONAR = true;
    constexpr bool SERIAL_DEBUG_MOTOR_DRIVER = true;
}