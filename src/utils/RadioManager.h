#pragma once

#include <WiFi.h>
#include <NimBLEDevice.h> 
#include <Arduino.h>

class RadioManager {
public:
    // Boots radios on startup based on NVS Config
    static void initRadios();
    
    // Status checks
    static bool isWiFiReady();
    static bool isBluetoothReady();
    static bool isBleConnected(); // NEW: Checks if the Python Bridge is listening

    // Dynamic Command Line Controls
    static void connectWiFi(String ssid, String password);
    static void disconnectWiFi();
    static void connectBluetooth(String name);
    static void disconnectBluetooth();
    
    // NEW: The Telemetry Blaster
    static void broadcastBLE(const uint8_t* data, size_t length);
};