#include "utils/RemoteLogger.h"
#include "utils/RadioManager.h"
#include "config/ConfigurationManager.h"

// ==========================================
// ALLOCATE THE STATIC GATEKEEPER MEMORY
// ==========================================
volatile int RemoteLogger::activeWebSocketClients = 0;
volatile unsigned long RemoteLogger::lastConnectTime = 0;

// 1. UPDATE THE CONSTRUCTOR: 
// Initialize the wifiSink with references to the websocket and the gatekeeper variables
RemoteLogger::RemoteLogger(int port) 
    : webSocket(port), 
      currentMode(0), 
      isBluetoothConnected(false),
      wifiSink(webSocket, activeWebSocketClients, lastConnectTime) 
{
    // Constructor body remains empty
}

// 2. ADD THE REGISTRATION METHOD
void RemoteLogger::registerSink(ITelemetrySink* newSink) {
    if (sinkCount < MAX_SINKS) {
        activeSinks[sinkCount] = newSink;
        sinkCount++;
    }
}

void RemoteLogger::beginSerial() {
    // The OS is running now. It is safe to allocate kernel objects!
    
    currentMode = SysConfig.ACTIVE_DEBUG_MODE;
    if (currentMode & SysConfig.DEBUG_USB) {
        Serial.begin(SysConfig.SERIAL_BAUD_RATE);
        delay(3000); 
    }
}

// 4. ADD THE JSON PUBLISHER
void RemoteLogger::publishTelemetry(const volatile GlobalDataBank& robotData, const char* mode, bool brainActive) {
    if (currentMode == SysConfig.DEBUG_ACTIVE) return;
    
    JsonDocument doc; 
    
    // --- 1. THE FAST PHYSICS DATA ---
    doc["yaw"]   = (float)robotData.physics.imuAngles.yaw;
    doc["pitch"] = (float)robotData.physics.imuAngles.pitch;
    doc["roll"]  = (float)robotData.physics.imuAngles.roll;
    doc["sonar"] = (float)robotData.sensors.distanceCM;
    doc["mode"]  = mode;
    doc["brain"] = brainActive;

    // --- 2. THE SYSTEM & NETWORK DATA ---
    if (WiFi.status() == WL_CONNECTED) {
        doc["wifi_status"] = "connected";
        doc["wifi_ssid"]   = WiFi.SSID();
        doc["wifi_rssi"]   = WiFi.RSSI(); // Excellent for tracking dead zones!
        doc["ipv4"]        = WiFi.localIP().toString();
    } else {
        // Check our NVS Config to see if it's intentionally off, or just disconnected
        doc["wifi_status"] = SysConfig.WIFI_ACTIVE ? "disconnected" : "off";
    }

    // Check BT status from our NVS Config
    doc["bt_status"] = SysConfig.BT_ACTIVE ? "on" : "off";

    // --- 3. THE SAFETY BUFFER INCREASE ---
    // Increased from 256 to 512 to accommodate the massive string additions!
    char jsonBuffer[512];
    serializeJson(doc, jsonBuffer);

    // // Blast it out to any sink that is currently registered and ready!
    // for (int i = 0; i < sinkCount; i++) {
    //     if (activeSinks[i] != nullptr && activeSinks[i]->isReady()) {
    //         activeSinks[i]->transmit(jsonBuffer);
    //     }
    // }
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

    // Register USB if active
    if (currentMode & SysConfig.DEBUG_USB) {
        registerSink(&usbSink);
    }

    // Register WiFi if active
    if (currentMode & SysConfig.DEBUG_WIFI) {
        webSocket.begin();
        webSocket.onEvent(webSocketEvent); 
        registerSink(&wifiSink);
        
        if (currentMode & SysConfig.DEBUG_USB) {
            Serial.println("WIFI: WebSocket Server Open");
        }
    }
}

void RemoteLogger::handleClient() {
    if (currentMode == SysConfig.DEBUG_ACTIVE) return;
    
    // THE FIREWALL: Never touch the websocket if the router is dead!
    if ((currentMode & SysConfig.DEBUG_WIFI) && WiFi.status() == WL_CONNECTED) {
        webSocket.loop(); 
    }
}

void RemoteLogger::print(const char* message) {
    if (currentMode == SysConfig.DEBUG_ACTIVE) return;
    if ((currentMode & SysConfig.DEBUG_USB) && Serial) Serial.print(message);
    // REMOVED WEBSOCKET BROADCAST
}

void RemoteLogger::println(const char* message) {
    if (currentMode == SysConfig.DEBUG_ACTIVE) return;
    if ((currentMode & SysConfig.DEBUG_USB) && Serial) Serial.println(message);
    // REMOVED WEBSOCKET BROADCAST
}

void RemoteLogger::printf(const char* format, ...) {
    if (currentMode == SysConfig.DEBUG_ACTIVE) return;

    char buffer[512]; 
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if ((currentMode & SysConfig.DEBUG_USB) && Serial) Serial.print(buffer);
    // REMOVED WEBSOCKET BROADCAST
}