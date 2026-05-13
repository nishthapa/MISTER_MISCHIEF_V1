#include "utils/RadioManager.h"
#include "config/WiFiConfig.h"
#include "config/BluetoothConfig.h"
#include "config/DebugConfig.h" // Needed just to check if we should print boot logs to USB

void RadioManager::initRadios() {
    bool printLogs = (DebugConfig::ACTIVE_DEBUG_MODE & DebugConfig::DEBUG_USB);

    if (printLogs) {
        Serial.println("\n=== RADIO INFRASTRUCTURE BOOT ===");
    }

    // 1. BOOT WIFI
    if (NetworkConfig::ENABLE_WIFI) {
        if (printLogs) {
            Serial.print("[WIFI] Connecting to: ");
            Serial.println(NetworkConfig::WIFI_SSID);
        }

        WiFi.begin(NetworkConfig::WIFI_SSID, NetworkConfig::WIFI_PASSWORD);
        
        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 30) {
            delay(500);
            if (printLogs) Serial.print(".");
            attempts++;
        }
        
        if (printLogs) {
            if (WiFi.status() == WL_CONNECTED) {
                Serial.print("\n[WIFI] ONLINE | IP: ");
                Serial.println(WiFi.localIP());
            } else {
                Serial.println("\n[WIFI] FAILED TO CONNECT");
            }
        }
    } else {
        if (printLogs) Serial.println("[WIFI] Disabled in Config.");
    }

    // 2. BOOT BLUETOOTH
    if (BluetoothConfig::ENABLE_BLUETOOTH) {
        if (printLogs) {
            Serial.print("[BLUETOOTH] Advertising as: ");
            Serial.println(BluetoothConfig::DEVICE_NAME);
        }
        
        // BLEDevice::init(BluetoothConfig::DEVICE_NAME);
        // ... Start BLE Server ...
    } else {
         if (printLogs) Serial.println("[BLUETOOTH] Disabled in Config.");
    }

    if (printLogs) {
        Serial.println("=================================\n");
    }
}

bool RadioManager::isWiFiReady() {
    return NetworkConfig::ENABLE_WIFI && (WiFi.status() == WL_CONNECTED);
}

bool RadioManager::isBluetoothReady() {
    return BluetoothConfig::ENABLE_BLUETOOTH; // && BLEServer->isCreated()
}