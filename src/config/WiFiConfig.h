#pragma once

namespace NetworkConfig {
    // === 1. MASTER INFRASTRUCTURE SWITCH ===
    // true = Turns on the radio and connects to the router
    // false = Total radio silence (Saves battery)
    // NOTE: If this is false, WiFi Telemetry will fail safely even if requested in DebugConfig!
    constexpr bool ENABLE_WIFI = true; 

    // === 2. SERVICE SWITCHES ===
    // Future features will go here! 
    // constexpr bool ENABLE_WEB_DASHBOARD = false;
    // constexpr bool ENABLE_OTA_UPDATES = false;

    // === CREDENTIALS ===
    constexpr char WIFI_SSID[] = "Airtel_Hevnoraak";
    constexpr char WIFI_PASSWORD[] = "Kalimpong@321";
    constexpr int TELNET_PORT = 23;
}