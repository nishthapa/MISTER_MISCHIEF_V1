#pragma once
#include <WiFi.h>
#include <stdarg.h>
#include <stdint.h> // For the uint8_t type

class RemoteLogger {
private:
    WiFiServer telnetServer;
    WiFiClient activeClient;
    
    // We use a boolean to track if a BLE device is connected
    bool isBluetoothConnected; 

    // The logger's internal memory of what is actually physically possible
    uint8_t currentMode; 

public:
    RemoteLogger(int port);
    
    // Stage 1: Boots the physical USB serial port instantly
    void beginSerial();
    
    // Stage 2: Connects to the requested infrastructure and prints the boot report
    void bindRadios();
    
    // Checks for new incoming PC/Phone connections
    void handleClient();
    
    // The drop-in replacements for Serial.print
    void print(const char* message);
    void println(const char* message);
    void printf(const char* format, ...);
};

// This tells all other files that "logger" exists globally!
extern RemoteLogger logger;