#include "utils/RemoteLogger.h"
#include "utils/RadioManager.h"
#include "config/ConfigurationManager.h"

// ==========================================
// ALLOCATE THE STATIC GATEKEEPER MEMORY
// ==========================================
volatile int RemoteLogger::activeWebSocketClients = 0;
volatile unsigned long RemoteLogger::lastConnectTime = 0;

// Instantiate the WebSocket server on the port passed by main.cpp (usually 81 for WS)
RemoteLogger::RemoteLogger(int port) : webSocket(port), currentMode(0), isBluetoothConnected(false) {}

void RemoteLogger::beginSerial() {
    currentMode = Config.ACTIVE_DEBUG_MODE;
    if (currentMode & Config.DEBUG_USB) {
        Serial.begin(Config.SERIAL_BAUD_RATE);
        delay(3000); 
    }
}

// THE EVENT HANDLER
void RemoteLogger::webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.printf("[WIFI] Client #%u Disconnected!\n", num);
            if (activeWebSocketClients > 0) activeWebSocketClients--;
            break;
        case WStype_CONNECTED:
            Serial.printf("[WIFI] Client #%u Connected! Pausing telemetry for 1 second...\n", num);
            activeWebSocketClients++;
            lastConnectTime = millis(); // Start the 1-second cooldown timer!
            break;
        case WStype_TEXT:
            break;
    }
}

void RemoteLogger::bindRadios() { 
    if ((currentMode & Config.DEBUG_WIFI) && !RadioManager::isWiFiReady()) {
        currentMode &= ~Config.DEBUG_WIFI;
        currentMode |= Config.DEBUG_USB;   
    }
    
    if (currentMode & Config.DEBUG_USB) {
        Serial.println("=== TELEMETRY ROUTER ONLINE ===");
        Serial.println("[USB] ONLINE");
        Serial.print("[WIFI] ");
        if (currentMode & Config.DEBUG_WIFI) Serial.println("WEBSOCKET ROUTED");
        else Serial.println("OFF / UNAVAILABLE");
        Serial.println("===============================\n");
    }

    if (currentMode == Config.DEBUG_ACTIVE) return;

    if (currentMode & Config.DEBUG_WIFI) {
        webSocket.begin();
        webSocket.onEvent(webSocketEvent); // Bind the event handler
        if (currentMode & Config.DEBUG_USB) {
            Serial.println("WIFI: WebSocket Server Open");
        }
    }
}

void RemoteLogger::handleClient() {
    if (currentMode == Config.DEBUG_ACTIVE) return;
    
    // THE FIREWALL: Never touch the websocket if the router is dead!
    if ((currentMode & Config.DEBUG_WIFI) && WiFi.status() == WL_CONNECTED) {
        webSocket.loop(); 
    }
}

void RemoteLogger::print(const char* message) {
    if (currentMode == Config.DEBUG_ACTIVE) return;
    if ((currentMode & Config.DEBUG_USB) && Serial) Serial.print(message);
    // REMOVED WEBSOCKET BROADCAST
}

void RemoteLogger::println(const char* message) {
    if (currentMode == Config.DEBUG_ACTIVE) return;
    if ((currentMode & Config.DEBUG_USB) && Serial) Serial.println(message);
    // REMOVED WEBSOCKET BROADCAST
}

void RemoteLogger::printf(const char* format, ...) {
    if (currentMode == Config.DEBUG_ACTIVE) return;

    char buffer[512]; 
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if ((currentMode & Config.DEBUG_USB) && Serial) Serial.print(buffer);
    // REMOVED WEBSOCKET BROADCAST
}

// === THE THREAD-SAFE TELEMETRY FIREWALL ===
void RemoteLogger::sendTelemetryJSON(const char* format, ...) {
    if (currentMode == Config.DEBUG_ACTIVE) return;

    char buffer[512]; 
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if ((currentMode & Config.DEBUG_USB) && Serial) Serial.print(buffer);
    
    // === THE GATEKEEPER ===
    if ((currentMode & Config.DEBUG_WIFI) && WiFi.status() == WL_CONNECTED) {
        // Only touch the WebSocket if WiFi is connected and if a client is FULLY connected AND the 1-second 
        // connection cooldown timer has finished. This guarantees the handshake is safe!
        if (activeWebSocketClients > 0 && (millis() - lastConnectTime > 1000)) {
            webSocket.broadcastTXT(buffer);
        }
    }
}