#pragma once
#include <stdint.h>

namespace DebugConfig {
    // 1. Define the Bitmask Values (Powers of 2)
    constexpr uint8_t DEBUG_OFF       = 0;       // 00000000
    constexpr uint8_t DEBUG_USB       = 1 << 0;  // 00000001 (Decimal 1)
    constexpr uint8_t DEBUG_WIFI      = 1 << 1;  // 00000010 (Decimal 2)
    constexpr uint8_t DEBUG_BLUETOOTH = 1 << 2;  // 00000100 (Decimal 4)

    // ========================================================
    // 2. THE MASTER DEBUG SWITCH
    // Combine modes using the '|' (OR) operator.
    // Example: DEBUG_USB | DEBUG_WIFI
    // To disable everything, just use: DEBUG_OFF
    // ========================================================
    
    constexpr uint8_t ACTIVE_DEBUG_MODE = DEBUG_USB | DEBUG_WIFI | DEBUG_BLUETOOTH; 
}