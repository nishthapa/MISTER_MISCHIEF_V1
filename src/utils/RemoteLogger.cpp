#include "utils/RemoteLogger.h"
//#include "config/DebugConfig.h"
#include "utils/RadioManager.h"
#include "config/ConfigurationManager.h"

RemoteLogger::RemoteLogger(int port) : telnetServer(port), currentMode(0), isBluetoothConnected(false) {}

void RemoteLogger::beginSerial() {
    // Grab the intended mode instantly on boot
    currentMode = Config.ACTIVE_DEBUG_MODE;

    // Boot USB instantly so other classes can use logger.print during setup!
    if (currentMode & Config.DEBUG_USB) {
        Serial.begin(Config.SERIAL_BAUD_RATE);
        delay(3000); // Give VS Code / Native USB time to connect
    }
}

void RemoteLogger::bindRadios() { 
    // 1. SILENT FALLBACK EVALUATION
    if ((currentMode & Config.DEBUG_WIFI) && !RadioManager::isWiFiReady()) {
        currentMode &= ~Config.DEBUG_WIFI;
        currentMode |= Config.DEBUG_USB;   
    }
    if ((currentMode & Config.DEBUG_BLUETOOTH) && !RadioManager::isBluetoothReady()) {
        currentMode &= ~Config.DEBUG_BLUETOOTH;
        currentMode |= Config.DEBUG_USB;   
    }

    // 2. PRINT BOOT BANNER 
    if (currentMode & Config.DEBUG_USB) {
        Serial.println("=== TELEMETRY ROUTER ONLINE ===");
        Serial.println("[USB] ONLINE");
        
        Serial.print("[WIFI] ");
        if (currentMode & Config.DEBUG_WIFI) Serial.println("ROUTED");
        else Serial.println("OFF / UNAVAILABLE");

        Serial.print("[BLUETOOTH] ");
        if (currentMode & Config.DEBUG_BLUETOOTH) Serial.println("ROUTED");
        else Serial.println("OFF / UNAVAILABLE");
        
        Serial.println("===============================\n");
    }

    if (currentMode == Config.DEBUG_ACTIVE) return;

    // 3. ATTACH TO RADIOS
    if (currentMode & Config.DEBUG_WIFI) {
        telnetServer.begin();
        if (currentMode & Config.DEBUG_USB) {
            Serial.println("WIFI: Telnet Socket Open (Port 23)");
        }
    }
}

void RemoteLogger::handleClient() {
    if (currentMode == Config.DEBUG_ACTIVE) return;

    if (currentMode & Config.DEBUG_WIFI) {
        if (telnetServer.hasClient()) {
            if (!activeClient || !activeClient.connected()) {
                if (activeClient) activeClient.stop();
                
                activeClient = telnetServer.available();
                
                // === THE ZERO-LAG FIX ===
                // This disables Nagle's Algorithm, forcing the router to transmit
                // telemetry instantly instead of buffering it into chunks.
                activeClient.setNoDelay(true); 

                if (currentMode & Config.DEBUG_USB) {
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
    if (currentMode == Config.DEBUG_ACTIVE) return;

    // THE FIX: Only print to USB if a cable is plugged in AND a terminal is listening
    if ((currentMode & Config.DEBUG_USB) && Serial) {
        Serial.print(message);
    }
    
    if ((currentMode & Config.DEBUG_WIFI) && activeClient && activeClient.connected()) {
        sendSanitizedToTelnet(activeClient, message);
    }

    // if ((currentMode & Config.DEBUG_BLUETOOTH) && isBluetoothConnected) {
    //     bleTxCharacteristic->setValue((uint8_t*)message, strlen(message));
    //     bleTxCharacteristic->notify();
    // }
}

void RemoteLogger::println(const char* message) {
    if (currentMode == Config.DEBUG_ACTIVE) return;

    // THE FIX: Check the physical connection first
    if ((currentMode & Config.DEBUG_USB) && Serial) {
        Serial.println(message);
    }
    
    if ((currentMode & Config.DEBUG_WIFI) && activeClient && activeClient.connected()) {
        activeClient.println(message);
    }

    // if ((currentMode & Config.DEBUG_BLUETOOTH) && isBluetoothConnected) {
    //     char buffer[256];
    //     snprintf(buffer, sizeof(buffer), "%s\n", message);
    //     bleTxCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
    //     bleTxCharacteristic->notify();
    // }
}

void RemoteLogger::printf(const char* format, ...) {
    if (currentMode == Config.DEBUG_ACTIVE) return;

    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // THE FIX: Check the physical connection first
    if ((currentMode & Config.DEBUG_USB) && Serial) {
        Serial.print(buffer);
    }
    
    if ((currentMode & Config.DEBUG_WIFI) && activeClient && activeClient.connected()) {
        sendSanitizedToTelnet(activeClient, buffer);
    }

    // if ((currentMode & Config.DEBUG_BLUETOOTH) && isBluetoothConnected) {
    //     bleTxCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
    //     bleTxCharacteristic->notify();
    // }
}