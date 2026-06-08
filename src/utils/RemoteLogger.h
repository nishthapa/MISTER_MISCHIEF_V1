#pragma once

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>       // <--- ADD THIS
#include "core/GlobalDataBus.h" // <--- ADD THIS
#include "comms/telemetry/TelemetrySinks.h"   // <--- INCLUDE YOUR NEW FILE HERE
#include <stdarg.h>
#include <stdint.h> // For the uint8_t type

class RemoteLogger {
private:
    //WiFiServer telnetServer;
    WebSocketsServer webSocket; // <--- REPLACE WiFiServer telnetServer;
    WiFiClient activeClient;
    
    // We use a boolean to track if a BLE device is connected
    bool isBluetoothConnected;

    // The logger's internal memory of what is actually physically possible
    uint8_t currentMode;

    // === THE GATEKEEPER SECRETS (Back in Private!) ===
    static volatile int activeWebSocketClients;
    static volatile unsigned long lastConnectTime;

    // Helper to handle client connections
    static void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

    // ==========================================
    // THE NEW TELEMETRY SINK ROUTER VARIABLES
    // ==========================================
    static constexpr int MAX_SINKS = 3;
    ITelemetrySink* activeSinks[MAX_SINKS] = {nullptr};
    int sinkCount = 0;

    // The physical sink instances owned by the logger
    USBSink usbSink;
    WebSocketSink wifiSink;

public:
    RemoteLogger(int port);

    // === ADD THESE 3 MISSING GETTERS ===
    WebSocketsServer& getServer() { return webSocket; }
    volatile int& getClientCount() { return activeWebSocketClients; }
    volatile unsigned long& getLastConnectTime() { return lastConnectTime; }
    // ===================================
    
    // Stage 1: Boots the physical USB serial port instantly
    void beginSerial();
    
    // Stage 2: Connects to the requested infrastructure and prints the boot report
    void bindRadios();
    
    // Checks for new incoming PC/Phone connections
    void handleClient();

    // The method to dynamically attach telemetry sinks
    void registerSink(ITelemetrySink* newSink);

    // === THE NEW JSON PUBLISHER ===
    void publishTelemetry(const volatile GlobalDataBank& robotData, const char* mode, bool brainActive);

    // For turning on/off from the command line
    void setMode(uint8_t newMode) { currentMode = newMode; }
    
    // The drop-in replacements for Serial.print
    void print(const char* message);
    void println(const char* message);
    void printf(const char* format, ...);

};

// This tells all other files that "logger" exists globally!
extern RemoteLogger logger;