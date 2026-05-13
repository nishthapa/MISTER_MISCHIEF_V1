#include "utils/RemoteLogger.h"
#include "config/DebugConfig.h"
#include "utils/RadioManager.h" 

RemoteLogger::RemoteLogger(int port) : telnetServer(port), currentMode(0), isBluetoothConnected(false) {}

void RemoteLogger::beginSerial() {
    // Grab the intended mode instantly on boot
    currentMode = DebugConfig::ACTIVE_DEBUG_MODE;

    // Boot USB instantly so other classes can use logger.print during setup!
    if (currentMode & DebugConfig::DEBUG_USB) {
        Serial.begin(115200);
        delay(3000); // Give VS Code / Native USB time to connect
    }
}

void RemoteLogger::bindRadios() { 
    // 1. SILENT FALLBACK EVALUATION
    if ((currentMode & DebugConfig::DEBUG_WIFI) && !RadioManager::isWiFiReady()) {
        currentMode &= ~DebugConfig::DEBUG_WIFI;
        currentMode |= DebugConfig::DEBUG_USB;   
    }
    if ((currentMode & DebugConfig::DEBUG_BLUETOOTH) && !RadioManager::isBluetoothReady()) {
        currentMode &= ~DebugConfig::DEBUG_BLUETOOTH;
        currentMode |= DebugConfig::DEBUG_USB;   
    }

    // 2. PRINT BOOT BANNER 
    if (currentMode & DebugConfig::DEBUG_USB) {
        Serial.println("=== TELEMETRY ROUTER ONLINE ===");
        Serial.println("[USB] ONLINE");
        
        Serial.print("[WIFI] ");
        if (currentMode & DebugConfig::DEBUG_WIFI) Serial.println("ROUTED");
        else Serial.println("OFF / UNAVAILABLE");

        Serial.print("[BLUETOOTH] ");
        if (currentMode & DebugConfig::DEBUG_BLUETOOTH) Serial.println("ROUTED");
        else Serial.println("OFF / UNAVAILABLE");
        
        Serial.println("===============================\n");
    }

    if (currentMode == DebugConfig::DEBUG_OFF) return;

    // 3. ATTACH TO RADIOS
    if (currentMode & DebugConfig::DEBUG_WIFI) {
        telnetServer.begin();
        if (currentMode & DebugConfig::DEBUG_USB) {
            Serial.println("WIFI: Telnet Socket Open (Port 23)");
        }
    }
}

void RemoteLogger::handleClient() {
    if (currentMode == DebugConfig::DEBUG_OFF) return;

    if (currentMode & DebugConfig::DEBUG_WIFI) {
        if (telnetServer.hasClient()) {
            if (!activeClient || !activeClient.connected()) {
                if (activeClient) activeClient.stop();
                
                activeClient = telnetServer.available();
                
                // === THE ZERO-LAG FIX ===
                // This disables Nagle's Algorithm, forcing the router to transmit
                // telemetry instantly instead of buffering it into chunks.
                activeClient.setNoDelay(true); 

                if (currentMode & DebugConfig::DEBUG_USB) {
                    Serial.println("\n[SYSTEM] WiFi Client Connected!\n");
                }
            } else {
                telnetServer.available().stop();
            }
        }
    }
}

// ========================================================
// THE NEW MIDDLEWARE SANITIZER (Network Optimized)
// ========================================================
void sendSanitizedToTelnet(WiFiClient& client, const char* text) {
    String sanitized = "";
    sanitized.reserve(128); // Pre-allocate RAM to prevent fragmentation
    char lastChar = '\0';
    
    // Build the packet entirely in fast memory
    while (*text) {
        if (*text == '\n' && lastChar != '\r') sanitized += '\r';
        sanitized += *text;
        lastChar = *text;
        text++;
    }
    
    // Blast the entire string across the network in ONE packet
    client.print(sanitized); 
    client.flush(); // Force immediate network transmission
}
// ========================================================

void RemoteLogger::print(const char* message) {
    if (currentMode == DebugConfig::DEBUG_OFF) return;

    // THE FIX: Only print to USB if a cable is plugged in AND a terminal is listening
    if ((currentMode & DebugConfig::DEBUG_USB) && Serial) {
        Serial.print(message);
    }
    
    if ((currentMode & DebugConfig::DEBUG_WIFI) && activeClient && activeClient.connected()) {
        sendSanitizedToTelnet(activeClient, message);
    }

    // if ((currentMode & DebugConfig::DEBUG_BLUETOOTH) && isBluetoothConnected) {
    //     bleTxCharacteristic->setValue((uint8_t*)message, strlen(message));
    //     bleTxCharacteristic->notify();
    // }
}

void RemoteLogger::println(const char* message) {
    if (currentMode == DebugConfig::DEBUG_OFF) return;

    // THE FIX: Check the physical connection first
    if ((currentMode & DebugConfig::DEBUG_USB) && Serial) {
        Serial.println(message);
    }
    
    if ((currentMode & DebugConfig::DEBUG_WIFI) && activeClient && activeClient.connected()) {
        activeClient.println(message);
    }

    // if ((currentMode & DebugConfig::DEBUG_BLUETOOTH) && isBluetoothConnected) {
    //     char buffer[256];
    //     snprintf(buffer, sizeof(buffer), "%s\n", message);
    //     bleTxCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
    //     bleTxCharacteristic->notify();
    // }
}

void RemoteLogger::printf(const char* format, ...) {
    if (currentMode == DebugConfig::DEBUG_OFF) return;

    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // THE FIX: Check the physical connection first
    if ((currentMode & DebugConfig::DEBUG_USB) && Serial) {
        Serial.print(buffer);
    }
    
    if ((currentMode & DebugConfig::DEBUG_WIFI) && activeClient && activeClient.connected()) {
        sendSanitizedToTelnet(activeClient, buffer);
    }

    // if ((currentMode & DebugConfig::DEBUG_BLUETOOTH) && isBluetoothConnected) {
    //     bleTxCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
    //     bleTxCharacteristic->notify();
    // }
}