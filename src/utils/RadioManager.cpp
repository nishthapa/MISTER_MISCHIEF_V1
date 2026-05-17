#include "utils/RadioManager.h"
#include "config/WiFiConfig.h"       // Kept for backward compatibility if needed
#include "config/BluetoothConfig.h" 
#include "config/DebugConfig.h" 
#include "config/ConfigurationManager.h" // The new NVS Hook!

void RadioManager::initRadios() {
    bool printLogs = (DebugConfig::ACTIVE_DEBUG_MODE & DebugConfig::DEBUG_USB);

    if (printLogs) {
        Serial.println("\n=== RADIO INFRASTRUCTURE BOOT ===");
    }

    // 1. BOOT WIFI (Using NVS Config!)
    if (Config.WIFI_ACTIVE) {
        if (printLogs) {
            Serial.print("[WIFI] Connecting to: ");
            Serial.println(Config.WIFI_SSID);
        }

        WiFi.begin(Config.WIFI_SSID.c_str(), Config.WIFI_PASSWORD.c_str());
        
        int attempts = 0;
        // EXACT RETRY LOGIC PRESERVED
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

    // 2. BOOT BLUETOOTH (Using NVS Config!)
    if (Config.BT_ACTIVE) {
        if (printLogs) {
            Serial.print("[BLUETOOTH] Advertising as: ");
            Serial.println(Config.BT_NAME);
        }
        
        // BLEDevice::init(Config.BT_NAME.c_str());
        // ... Start BLE Server ...
    } else {
         if (printLogs) Serial.println("[BLUETOOTH] Disabled in Config.");
    }

    if (printLogs) {
        Serial.println("=================================\n");
    }
}

// --- NEW DYNAMIC COMMAND LINE CONTROLS ---

void RadioManager::connectWiFi(String ssid, String password) {
    bool printLogs = (DebugConfig::ACTIVE_DEBUG_MODE & DebugConfig::DEBUG_USB);
    if (ssid == "") return;
    
    // Disconnect if already connected before trying new credentials
    if (WiFi.status() == WL_CONNECTED) {
        disconnectWiFi();
    }

    if (printLogs) {
        Serial.print("\n[WIFI] CLI Attempting to connect to: ");
        Serial.println(ssid);
    }

    WiFi.begin(ssid.c_str(), password.c_str());
    
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
}

void RadioManager::disconnectWiFi() {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}

void RadioManager::connectBluetooth(String name) {
    // BLEDevice::init(name.c_str());
    // ... Start BLE Server ...
}

void RadioManager::disconnectBluetooth() {
    // BLEDevice::deinit();
}

bool RadioManager::isWiFiReady() {
    return Config.WIFI_ACTIVE && (WiFi.status() == WL_CONNECTED);
}

bool RadioManager::isBluetoothReady() {
    return Config.BT_ACTIVE; // && BLEServer->isCreated();
}