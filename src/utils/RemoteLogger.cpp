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

// ========================================================
// THREAD-SAFE available()
// ========================================================
int RemoteLogger::available() {
    int count = 0;
    if (currentMode == 0) return 0;
    
    if (xSemaphoreTake(txMutex, portMAX_DELAY)) {
        if ((currentMode & Config.DEBUG_WIFI) && activeClient && activeClient.connected()) {
            count = activeClient.available();
        }
        xSemaphoreGive(txMutex);
    }
    return count;
}

// ========================================================
// THREAD-SAFE read()
// ========================================================
int RemoteLogger::read() {
    int data = -1;
    if (currentMode == 0) return -1;
    
    if (xSemaphoreTake(txMutex, portMAX_DELAY)) {
        if ((currentMode & Config.DEBUG_WIFI) && activeClient && activeClient.connected()) {
            data = activeClient.read();
        }
        xSemaphoreGive(txMutex);
    }
    return data;
}

// ========================================================
// THREAD-SAFE handleClient()
// ========================================================
void RemoteLogger::handleClient() {
    if (currentMode == Config.DEBUG_ACTIVE) return;

    if (currentMode & Config.DEBUG_WIFI) {
        if (telnetServer.hasClient()) {
            WiFiClient newClient = telnetServer.available();
            bool connectionSuccessful = false;

            // === CRITICAL SECTION ===
            if (xSemaphoreTake(txMutex, portMAX_DELAY)) {
                if (!activeClient || !activeClient.connected()) {
                    if (activeClient) activeClient.stop();
                    activeClient = newClient;
                    activeClient.setNoDelay(true);
                    connectionSuccessful = true;
                } else {
                    // We already have an active client, reject the new one
                    newClient.stop();
                }
                xSemaphoreGive(txMutex);
            }
            // ========================

            // Print OUTSIDE the mutex to prevent a total system deadlock!
            if (connectionSuccessful) {
                logger.println("[NETWORK] Telnet Client connected."); 
                if (currentMode & Config.DEBUG_USB) {
                    Serial.println("\n[SYSTEM] WiFi Client Connected!\n");
                }
            }
        }
    }
}

// ========================================================
// THE SAFE SANITIZER (Network Optimized AND HEAP-ALLOCATION FREE)
// ========================================================
void sendSanitizedToTelnet(WiFiClient& client, const char* text) {
    // Allocate a fast, static-sized RAM buffer on the stack (No Heap allocation!)
    char buffer[512]; 
    size_t bufIndex = 0;
    char lastChar = '\0';

    while (*text && bufIndex < 510) {
        if (*text == '\n' && lastChar != '\r') {
            buffer[bufIndex++] = '\r';
        }
        buffer[bufIndex++] = *text;
        lastChar = *text;
        text++;
    }
    buffer[bufIndex] = '\0'; // Null-terminate just to be safe
    
    // Blast the buffer to LwIP. Let the Wi-Fi driver handle the timing!
    client.print(buffer);
    
    // NOTE: client.flush() HAS BEEN REMOVED! 
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
            sendSanitizedToTelnet(activeClient, message); // <-- Wired up!
        }
        xSemaphoreGive(txMutex);
    }
}

// ========================================================
// THREAD-SAFE print (overload for single char)
// ========================================================
/*void RemoteLogger::print(char c) {
    if (currentMode == 0) return; // 0 means all telemetry is off

    // Ask for the Mutex. If Core 1 is using it, wait here until it's done!
    if (xSemaphoreTake(txMutex, portMAX_DELAY)) {
        if ((currentMode & Config.DEBUG_USB) && Serial) {
            Serial.print(c);
        }
        if ((currentMode & Config.DEBUG_WIFI) && activeClient && activeClient.connected()) {
            //activeClient.print(c);
            sendSanitizedToTelnet(activeClient, &c); // <-- Wired up!
        }
        xSemaphoreGive(txMutex); // Release the Mutex so the other Core can talk
    }
}*/

void RemoteLogger::print(char c) {
    if (currentMode == 0) return; // 0 means all telemetry is off

    // Ask for the Mutex. If Core 1 is using it, wait here until it's done!
    if (xSemaphoreTake(txMutex, portMAX_DELAY)) {
        if ((currentMode & Config.DEBUG_USB) && Serial) {
            Serial.print(c);
        }
        if ((currentMode & Config.DEBUG_WIFI) && activeClient && activeClient.connected()) {
            // Wrap the single char into a null-terminated C-string
            char charStr[2] = {c, '\0'}; 
            
            // Send it through the sanitizer!
            sendSanitizedToTelnet(activeClient, charStr); 
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
            sendSanitizedToTelnet(activeClient, message); // <-- Wired up!
            activeClient.print("\r\n"); // Ensure telnet terminals step down a line properly
            //activeClient.flush();
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
            sendSanitizedToTelnet(activeClient, buffer);  // <-- Wired up!
        }
        xSemaphoreGive(txMutex);
    }
}