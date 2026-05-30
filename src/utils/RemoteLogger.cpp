#include "utils/RemoteLogger.h"
#include "utils/RadioManager.h"
#include "config/ConfigurationManager.h"

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
            break;
        case WStype_CONNECTED:
            Serial.printf("[WIFI] Client #%u Connected!\n", num);
            break;
        case WStype_TEXT:
            // If you ever want to send commands FROM the dashboard TO the robot, handle payload here
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
    
    if (currentMode & Config.DEBUG_WIFI) {
        webSocket.loop(); // Keeps the WebSocket alive and processes incoming handshakes
    }
}

void RemoteLogger::print(const char* message) {
    if (currentMode == Config.DEBUG_ACTIVE) return;
    if ((currentMode & Config.DEBUG_USB) && Serial) Serial.print(message);
    if (currentMode & Config.DEBUG_WIFI) webSocket.broadcastTXT(message);
}

void RemoteLogger::println(const char* message) {
    if (currentMode == Config.DEBUG_ACTIVE) return;
    if ((currentMode & Config.DEBUG_USB) && Serial) Serial.println(message);
    
    if (currentMode & Config.DEBUG_WIFI) {
        // Broadcast appends nothing, so we just send the message
        webSocket.broadcastTXT(message);
    }
}

// THE STACK-ALLOCATED FIX (No more Core 0 Panics!)
void RemoteLogger::printf(const char* format, ...) {
    if (currentMode == Config.DEBUG_ACTIVE) return;

    char buffer[512]; // Fixed stack buffer. Safe from heap fragmentation!
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if ((currentMode & Config.DEBUG_USB) && Serial) Serial.print(buffer);
    if (currentMode & Config.DEBUG_WIFI) webSocket.broadcastTXT(buffer);
}