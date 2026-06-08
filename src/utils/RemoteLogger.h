#pragma once

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <stdarg.h>
#include <stdint.h>

class RemoteLogger {
private:
    WebSocketsServer webSocket; 
    bool isBluetoothConnected;
    uint8_t currentMode;

    // === THE GATEKEEPER SECRETS ===
    static volatile int activeWebSocketClients;
    static volatile unsigned long lastConnectTime;

    // Helper to handle client connections
    static void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

public:
    RemoteLogger(int port);
    
    // Gives the Telemetry Sinks access to the raw server for binary blasting
    WebSocketsServer& getServer() { return webSocket; }
    volatile int& getClientCount() { return activeWebSocketClients; }
    volatile unsigned long& getLastConnectTime() { return lastConnectTime; }

    // Boot stages
    void beginSerial();
    void bindRadios();
    void handleClient();

    // For turning on/off from the command line
    void setMode(uint8_t newMode) { currentMode = newMode; }
    
    // The drop-in replacements for Serial.print (The Human Pipeline)
    void print(const char* message);
    void println(const char* message);
    void printf(const char* format, ...);
};

// This tells all other files that "logger" exists globally!
extern RemoteLogger logger;