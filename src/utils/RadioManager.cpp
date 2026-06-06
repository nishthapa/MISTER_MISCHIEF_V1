#include "utils/RadioManager.h" 
#include "config/ConfigurationManager.h" // The new NVS Hook!
#include "core/GlobalSensorState.h" // We need access to TeleopCommands!

// These exact UUIDs match the ones in the Android App's BleManager.kt
#define SERVICE_UUID        "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000ffe1-0000-1000-8000-00805f9b34fb"

NimBLEServer* pServer = NULL;

// ====================================================
// BLE CALLBACKS: NETWORK STATE & FAILSAFES
// ====================================================
class BleServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        TeleopCommands.isConnected = true;
        Serial.println("\n[BLE] Remote Control App Connected!");
    }

    void onDisconnect(BLEServer* pServer) {
        TeleopCommands.isConnected = false;
        
        // FAILSAFE: Zero out all kinematic commands immediately if phone disconnects!
        TeleopCommands.joyX = 0.0f;
        TeleopCommands.joyY = 0.0f;
        TeleopCommands.usePIDDrive = false;
        
        Serial.println("\n[BLE] Remote Control Disconnected. Failsafe triggered. Restarting advertising...");
        
        // Standard BLE protocol requires manually restarting advertising after a client drops
        pServer->startAdvertising(); 
    }
};

// ====================================================
// BLE CALLBACKS: HIGH SPEED COMMAND PARSER
// ====================================================
class BleCommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        std::string value = pCharacteristic->getValue();
        
        // We expect exactly 9 bytes: [Float X (4)] [Float Y (4)] [PID Bool (1)]
        if (value.length() == 9) {
            const uint8_t* payload = (const uint8_t*)value.data();
            
            float rxX, rxY;
            // memcpy is mandatory here to safely cast raw little-endian bytes back into floats
            memcpy(&rxX, &payload[0], sizeof(float));
            memcpy(&rxY, &payload[4], sizeof(float));
            bool rxPID = (payload[8] != 0);

            // Push directly to the cross-core memory bank for the physics thread to consume
            TeleopCommands.joyX = rxX;
            TeleopCommands.joyY = rxY;
            TeleopCommands.usePIDDrive = rxPID;
        }
    }
};

void RadioManager::initRadios() {
    bool printLogs = (Config.ACTIVE_DEBUG_MODE & Config.DEBUG_USB);

    if (printLogs) {
        Serial.println("\n=== RADIO INFRASTRUCTURE BOOT ===");
    }

    // 1. BOOT WIFI
    if (Config.WIFI_ACTIVE) {
        if (printLogs) {
            Serial.print("[WIFI] Connecting to: ");
            Serial.println(Config.WIFI_SSID);
        }

        WiFi.begin(Config.WIFI_SSID.c_str(), Config.WIFI_PASSWORD.c_str());
        
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
    if (Config.BT_ACTIVE) {
        if (printLogs) {
            Serial.print("[BLUETOOTH] Advertising as: ");
            Serial.println(Config.BT_NAME);
        }
        
        // 1. Initialize the NimBLE Hardware Radio
        NimBLEDevice::init(Config.BT_NAME.c_str());
        
        // 2. Create the Server and attach connection failsafes
        pServer = NimBLEDevice::createServer();
        pServer->setCallbacks(new BleServerCallbacks());

        // 3. Create the Custom Service
        NimBLEService *pService = pServer->createService(SERVICE_UUID);

        // 4. Create the Characteristic using the new NIMBLE_PROPERTY syntax
        NimBLECharacteristic *pCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID,
            NIMBLE_PROPERTY::WRITE | 
            NIMBLE_PROPERTY::WRITE_NR // WRITE_NO_RESPONSE for zero latency
        );

        pCharacteristic->setCallbacks(new BleCommandCallbacks());
        
        // 5. Start the Server (This automatically starts all attached services natively)
        pServer->start();

        // 6. Configure advertising
        NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(SERVICE_UUID);
        pAdvertising->start();

        if (printLogs) Serial.println("[BLUETOOTH] GATT Server Online and Broadcasting.");
        
    } else {
         if (printLogs) Serial.println("[BLUETOOTH] Disabled in Config.");
    }

    if (printLogs) {
        Serial.println("=================================\n");
    }
}

// --- NEW DYNAMIC COMMAND LINE CONTROLS ---

void RadioManager::connectWiFi(String ssid, String password) {
    bool printLogs = (Config.ACTIVE_DEBUG_MODE & Config.DEBUG_USB);
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
    NimBLEDevice::deinit();
}

bool RadioManager::isWiFiReady() {
    return Config.WIFI_ACTIVE && (WiFi.status() == WL_CONNECTED);
}

bool RadioManager::isBluetoothReady() {
    return Config.BT_ACTIVE; // && BLEServer->isCreated();
}