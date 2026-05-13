#include "utils/RemoteLogger.h"
#include "config/DebugConfig.h"
#include "utils/RadioManager.h" // <-- The Logger now talks to the Radio Manager

RemoteLogger::RemoteLogger(int port) : telnetServer(port), currentMode(0), isBluetoothConnected(false) {}

void RemoteLogger::init() { // <-- Notice we don't need to pass SSID/Password anymore!
    currentMode = DebugConfig::ACTIVE_DEBUG_MODE;

    // 1. SILENT FALLBACK EVALUATION
    // The logger asks the RadioManager if the physical pipes are actually open
    if ((currentMode & DebugConfig::DEBUG_WIFI) && !RadioManager::isWiFiReady()) {
        currentMode &= ~DebugConfig::DEBUG_WIFI;
        currentMode |= DebugConfig::DEBUG_USB;   
    }
    if ((currentMode & DebugConfig::DEBUG_BLUETOOTH) && !RadioManager::isBluetoothReady()) {
        currentMode &= ~DebugConfig::DEBUG_BLUETOOTH;
        currentMode |= DebugConfig::DEBUG_USB;   
    }

    // 2. BOOT USB (If requested or fallback triggered)
    if (currentMode & DebugConfig::DEBUG_USB) {
        // Serial.begin(115200);
        // now handled in main.cpp setup() to ensure it's ready for the boot report
        // delay(500);
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

    // 3. ATTACH TO RADIOS (We don't boot them, we just attach our sockets to them!)
    if (currentMode & DebugConfig::DEBUG_WIFI) {
        telnetServer.begin();
        if (currentMode & DebugConfig::DEBUG_USB) {
            Serial.println("WIFI: Telnet Socket Open (Port 23)");
        }
    }

    if (currentMode & DebugConfig::DEBUG_BLUETOOTH) {
        // bleTxCharacteristic = ... attach to existing BLE server ...
    }
}

void RemoteLogger::handleClient() {
    // Master Kill Switch
    if (currentMode == DebugConfig::DEBUG_OFF) return;

    // Handle WiFi Client Connections
    if (currentMode & DebugConfig::DEBUG_WIFI) {
        if (telnetServer.hasClient()) {
            if (!activeClient || !activeClient.connected()) {
                if (activeClient) activeClient.stop();
                activeClient = telnetServer.available();
                if (currentMode & DebugConfig::DEBUG_USB) {
                    Serial.println("\n[SYSTEM] WiFi Client Connected!\n");
                }
            } else {
                telnetServer.available().stop();
            }
        }
    }

    // Handle Bluetooth Client Connections (Pseudo-code update based on BLE callbacks)
    // if (currentMode & DebugConfig::DEBUG_BLUETOOTH) {
    //     isBluetoothConnected = BLEServer->getConnectedCount() > 0;
    // }
}

// ========================================================
// THE NEW MIDDLEWARE SANITIZER
// ========================================================
void sendSanitizedToTelnet(WiFiClient& client, const char* text) {
    char lastChar = '\0';
    while (*text) {
        // If we hit a newline, and the character before it WAS NOT a carriage return...
        if (*text == '\n' && lastChar != '\r') {
            client.print('\r'); // Inject the missing Carriage Return!
        }
        client.print(*text);
        lastChar = *text;
        text++;
    }
}
// ========================================================

void RemoteLogger::print(const char* message) {
    if (currentMode == DebugConfig::DEBUG_OFF) return;

    if (currentMode & DebugConfig::DEBUG_USB) {
        Serial.print(message);
    }
    
    if ((currentMode & DebugConfig::DEBUG_WIFI) && activeClient && activeClient.connected()) {
        // Route through our new sanitizer!
        sendSanitizedToTelnet(activeClient, message);
    }

    // if ((currentMode & DebugConfig::DEBUG_BLUETOOTH) && isBluetoothConnected) {
    //     bleTxCharacteristic->setValue((uint8_t*)message, strlen(message));
    //     bleTxCharacteristic->notify();
    // }
}

void RemoteLogger::println(const char* message) {
    if (currentMode == DebugConfig::DEBUG_OFF) return;

    if (currentMode & DebugConfig::DEBUG_USB) {
        Serial.println(message);
    }
    
    if ((currentMode & DebugConfig::DEBUG_WIFI) && activeClient && activeClient.connected()) {
        // Fun Fact: println() naturally sends \r\n under the hood, 
        // so we don't need to sanitize this one!
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

    if (currentMode & DebugConfig::DEBUG_USB) {
        Serial.print(buffer);
    }
    
    if ((currentMode & DebugConfig::DEBUG_WIFI) && activeClient && activeClient.connected()) {
        // Route the formatted buffer through our sanitizer!
        sendSanitizedToTelnet(activeClient, buffer);
    }

    // if ((currentMode & DebugConfig::DEBUG_BLUETOOTH) && isBluetoothConnected) {
    //     bleTxCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
    //     bleTxCharacteristic->notify();
    // }
}