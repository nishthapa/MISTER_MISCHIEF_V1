#include "utils/RemoteLogger.h"
//#include "config/DebugConfig.h"
#include "utils/RadioManager.h"
#include "config/ConfigurationManager.h"

RemoteLogger::RemoteLogger(int port) : telnetServer(port), currentMode(0), isBluetoothConnected(false) {}

void RemoteLogger::beginSerial() {
    txMutex = xSemaphoreCreateMutex(); // Create the mutex for synchronizing access to the WiFi client

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

// ========================================================
// INCOMING COMMAND PROCESSING (wireless TELNET)
// ========================================================
int RemoteLogger::available() {
    if ((currentMode & Config.DEBUG_WIFI) && activeClient && activeClient.connected()) {
        return activeClient.available();
    }
    return 0;
}

int RemoteLogger::read() {
    if ((currentMode & Config.DEBUG_WIFI) && activeClient && activeClient.connected()) {
        return activeClient.read();
    }
    return -1;
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
                logger.println("[NETWORK] Telnet Client connected."); 

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

// ========================================================
// THREAD-SAFE print
// ========================================================
void RemoteLogger::print(const char* message) {
    if (currentMode == 0) return;

    if (xSemaphoreTake(txMutex, portMAX_DELAY)) {
        if ((currentMode & Config.DEBUG_USB) && Serial) {
            Serial.print(message);
        }
        if ((currentMode & Config.DEBUG_WIFI) && activeClient && activeClient.connected()) {
            activeClient.print(message);
        }
        xSemaphoreGive(txMutex);
    }
}

// ========================================================
// THREAD-SAFE print (overload for single char)
// ========================================================
void RemoteLogger::print(char c) {
    if (currentMode == 0) return; // 0 means all telemetry is off

    // Ask for the Mutex. If Core 1 is using it, wait here until it's done!
    if (xSemaphoreTake(txMutex, portMAX_DELAY)) {
        if ((currentMode & Config.DEBUG_USB) && Serial) {
            Serial.print(c);
        }
        if ((currentMode & Config.DEBUG_WIFI) && activeClient && activeClient.connected()) {
            activeClient.print(c);
        }
        xSemaphoreGive(txMutex); // Release the Mutex so the other Core can talk
    }
}

// ========================================================
// THREAD-SAFE println
// ========================================================
void RemoteLogger::println(const char* message) {
    if (currentMode == 0) return;

    if (xSemaphoreTake(txMutex, portMAX_DELAY)) {
        if ((currentMode & Config.DEBUG_USB) && Serial) {
            Serial.println(message);
        }
        if ((currentMode & Config.DEBUG_WIFI) && activeClient && activeClient.connected()) {
            activeClient.println(message);
        }
        xSemaphoreGive(txMutex);
    }
}

// ========================================================
// THREAD-SAFE printf
// ========================================================
void RemoteLogger::printf(const char* format, ...) {
    if (currentMode == 0) return;

    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (xSemaphoreTake(txMutex, portMAX_DELAY)) {
        if ((currentMode & Config.DEBUG_USB) && Serial) {
            Serial.print(buffer);
        }
        if ((currentMode & Config.DEBUG_WIFI) && activeClient && activeClient.connected()) {
            activeClient.print(buffer);
        }
        xSemaphoreGive(txMutex);
    }
}