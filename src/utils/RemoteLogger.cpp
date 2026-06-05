#include "utils/RemoteLogger.h"
#include "utils/RadioManager.h"
#include "config/ConfigurationManager.h"

// ==========================================
// ALLOCATE THE STATIC GATEKEEPER MEMORY
// ==========================================
volatile int RemoteLogger::activeWebSocketClients = 0;
volatile unsigned long RemoteLogger::lastConnectTime = 0;

// Instantiate the WebSocket server on the port passed by main.cpp (usually 81 for WS)
// 1. Leave the constructor completely empty!
RemoteLogger::RemoteLogger(int port) : webSocket(port), currentMode(0), isBluetoothConnected(false) {
    // DO NOT CALL xQueueCreate HERE
    //telemetryQueue = xQueueCreate(10, 256); 
}

void RemoteLogger::beginSerial() {
    // The OS is running now. It is safe to allocate kernel objects!
    // We allocate space for 10 messages, each up to 256 bytes long.
    telemetryQueue = xQueueCreate(10, 256);
    
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

// === THE TELEMETRY PRODUCER (Runs on Core 1) ===
void RemoteLogger::sendTelemetryJSON(const char* format, ...) {
    if (currentMode == Config.DEBUG_ACTIVE) return;

    char buffer[256]; // Changed from 512 to 256 to exactly match the FREERTOS queue size!
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if ((currentMode & Config.DEBUG_USB) && Serial) Serial.print(buffer);

    // Instead of forcing a network transmission, just drop the string in the mailbox!
    // The '0' means do not block or wait if the queue is temporarily full.
    if (currentMode & Config.DEBUG_WIFI) {
        xQueueSend(telemetryQueue, buffer, 0); 
    }
}

// === THE TELEMETRY CONSUMER (Runs on Core 0) ===
void RemoteLogger::processQueue() {
    if (currentMode == Config.DEBUG_ACTIVE) return;
    
    char buffer[256];
    
    // THE GATEKEEPER: Only touch the WebSocket if it is safe to do so
    if ((currentMode & Config.DEBUG_WIFI) && WiFi.status() == WL_CONNECTED) {
        if (activeWebSocketClients > 0 && (millis() - lastConnectTime > 1000)) {
            
            // Empty the mailbox and transmit everything waiting inside
            while(xQueueReceive(telemetryQueue, buffer, 0) == pdTRUE) {
                webSocket.broadcastTXT(buffer);
            }
        } else {
            // If nobody is connected, secretly throw the mail in the trash so the box doesn't overflow
            while(xQueueReceive(telemetryQueue, buffer, 0) == pdTRUE) { /* discard */ }
        }
    }
}