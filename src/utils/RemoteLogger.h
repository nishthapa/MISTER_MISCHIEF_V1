#pragma once

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <stdarg.h>
#include <stdint.h> // For the uint8_t type
#include <freertos/FreeRTOS.h> // <--- NEW
#include <freertos/queue.h>    // <--- NEW

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

    QueueHandle_t telemetryQueue; // <--- THE BRIDGE MAILBOX for sending telemetry from the Brain to the logger task safely across cores!

    // Helper to handle client connections
    static void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

public:
    RemoteLogger(int port);
    
    // Stage 1: Boots the physical USB serial port instantly
    void beginSerial();
    
    // Stage 2: Connects to the requested infrastructure and prints the boot report
    void bindRadios();
    
    // Checks for new incoming PC/Phone connections
    void handleClient();

    // The Consumer function for the telemetry queue, called from the TelemetryTask loop
    void processQueue();

    // For turning on/off from the command line
    void setMode(uint8_t newMode) { currentMode = newMode; }
    
    // The drop-in replacements for Serial.print
    void print(const char* message);
    void println(const char* message);
    void printf(const char* format, ...);

    void sendTelemetryJSON(const char* format, ...);
};

// This tells all other files that "logger" exists globally!
extern RemoteLogger logger;