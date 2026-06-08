#include "utils/RemoteLogger.h"
#include "utils/RadioManager.h"
#include "config/ConfigurationManager.h"

// Allocate static memory
volatile int RemoteLogger::activeWebSocketClients = 0;
volatile unsigned long RemoteLogger::lastConnectTime = 0;

RemoteLogger::RemoteLogger(int port) 
    : webSocket(port), currentMode(0), isBluetoothConnected(false) {}

void RemoteLogger::beginSerial() {
    currentMode = SysConfig.ACTIVE_DEBUG_MODE;
    if (currentMode & SysConfig.DEBUG_USB) {
        Serial.begin(SysConfig.SERIAL_BAUD_RATE);
        delay(3000);
    }
}

void RemoteLogger::bindRadios() { 
    // THE FIREWALL: If WiFi is requested, but the modem is off/failed, downgrade to USB-only!
    if ((currentMode & SysConfig.DEBUG_WIFI) && !RadioManager::isWiFiReady()) {
        currentMode &= ~SysConfig.DEBUG_WIFI;
        currentMode |= SysConfig.DEBUG_USB;   
    }
    
    if (currentMode & SysConfig.DEBUG_USB) {
        Serial.println("=== TELEMETRY ROUTER ONLINE ===");
        Serial.println("[USB] ONLINE");
        Serial.print("[WIFI] ");
        if (currentMode & SysConfig.DEBUG_WIFI) Serial.println("WEBSOCKET ROUTED");
        else Serial.println("OFF / UNAVAILABLE");
        Serial.println("===============================\n");
    }

    if (currentMode == SysConfig.DEBUG_ACTIVE) return;

    // FIX: ONLY start the socket if the modem is confirmed alive!
    if (currentMode & SysConfig.DEBUG_WIFI) {
        webSocket.begin();
        webSocket.onEvent(webSocketEvent); 
    }
}

void RemoteLogger::handleClient() {
    if (currentMode == SysConfig.DEBUG_ACTIVE) return;
    
    if ((currentMode & SysConfig.DEBUG_WIFI) && WiFi.status() == WL_CONNECTED) {
        webSocket.loop(); 
    }
}

// ==========================================
// THE HUMAN PIPELINE (Text Logging)
// ==========================================
void RemoteLogger::print(const char* message) {
    if (currentMode == SysConfig.DEBUG_ACTIVE) return;
    
    if ((currentMode & SysConfig.DEBUG_USB) && Serial) {
        Serial.print(message);
    }
    // Broadcast human text over Wi-Fi
    if ((currentMode & SysConfig.DEBUG_WIFI) && activeWebSocketClients > 0) {
        webSocket.broadcastTXT(message);
    }
}

void RemoteLogger::println(const char* message) {
    if (currentMode == SysConfig.DEBUG_ACTIVE) return;
    
    if ((currentMode & SysConfig.DEBUG_USB) && Serial) {
        Serial.println(message);
    }
    // Broadcast human text over Wi-Fi
    if ((currentMode & SysConfig.DEBUG_WIFI) && activeWebSocketClients > 0) {
        webSocket.broadcastTXT(message);
        webSocket.broadcastTXT("\n");
    }
}

void RemoteLogger::printf(const char* format, ...) {
    if (currentMode == SysConfig.DEBUG_ACTIVE) return;

    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    print(buffer);
}

// ==========================================
// WEBSOCKET EVENT HANDLER
// ==========================================
void RemoteLogger::webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            if (activeWebSocketClients > 0) activeWebSocketClients--;
            Serial.printf("[%u] Disconnected!\n", num);
            break;
        case WStype_CONNECTED:
            activeWebSocketClients++;
            lastConnectTime = millis();
            Serial.printf("[%u] Connected!\n", num);
            break;
        case WStype_TEXT:
        case WStype_BIN:
        case WStype_ERROR:
        case WStype_FRAGMENT_TEXT_START:
        case WStype_FRAGMENT_BIN_START:
        case WStype_FRAGMENT:
        case WStype_FRAGMENT_FIN:
        case WStype_PING:
        case WStype_PONG:
            break;
    }
}