#pragma once

#include <WiFi.h>
#include <stdarg.h>
#include <stdint.h> // For the uint8_t type

// for thread safe WiFi TCP/IP handling
#include <freertos/FreeRTOS.h> // <-- ADD THIS
#include <freertos/semphr.h>   // <-- ADD THIS

class RemoteLogger {
private:
    WiFiServer telnetServer;
    WiFiClient activeClient;
    
    // We use a boolean to track if a BLE device is connected
    bool isBluetoothConnected; 

    // The logger's internal memory of what is actually physically possible
    uint8_t currentMode;

    SemaphoreHandle_t txMutex; // <-- for Cross core synchronization (sequential actions)

public:
    RemoteLogger(int port);
    
    // Stage 1: Boots the physical USB serial port instantly
    void beginSerial();
    
    // Stage 2: Connects to the requested infrastructure and prints the boot report
    void bindRadios();

    // Stage 3: Checks for and returns incoming network strings
    int available();
    int read();
  
    // Checks for new incoming PC/Phone connections
    void handleClient();

    // For turning on/off from the command line
    void setMode(uint8_t newMode) { currentMode = newMode; }
    
    // The drop-in replacements for Serial.print
    void print(const char* message);
    void print(char c);
    void println(const char* message);
    void printf(const char* format, ...);
};

// This tells all other files that "logger" exists globally!
extern RemoteLogger logger;