#pragma once

namespace BluetoothConfig {
    // === MASTER INFRASTRUCTURE SWITCH ===
    // true = Turns on the BLE radio
    // false = Total radio silence
    // NOTE: If this is false, Bluetooth Telemetry will fail safely even if requested in DebugConfig!
    constexpr bool ENABLE_BLUETOOTH = false; // Set to true to enable Bluetooth connectivity and bluetooth telemetry features. Set to false for total radio silence and maximum battery life.

    constexpr char DEVICE_NAME[] = "Mister_Mischief_BLE";
}