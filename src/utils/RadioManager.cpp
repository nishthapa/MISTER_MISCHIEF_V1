#include "utils/RadioManager.h" 
#include "config/ConfigurationManager.h" // The new NVS Hook!
#include "core/GlobalDataBus.h" // We need access to TeleopCommands!

// These exact UUIDs match the ones in the Android App's BleManager.kt
#define SERVICE_UUID        "0000ffe0-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID "0000ffe1-0000-1000-8000-00805f9b34fb" // High-speed driving commands
#define TOKEN_CHAR_UUID     "0000ffe2-0000-1000-8000-00805f9b34fb" // NEW: Explicit control flag

NimBLEServer* pServer = NULL;
NimBLECharacteristic* pBleCharacteristic = NULL; // NEW: Global pointer for the broadcaster

// ====================================================
// TOKEN MANAGER (NEW CALLBACK) FOR REMOTE DRIVING
// ====================================================
class BleTokenCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo& connInfo) override {
        NimBLEAttValue value = pCharacteristic->getValue();
        if (value.length() == 1) {
            uint8_t flag = value.data()[0];
            portENTER_CRITICAL(&teleopCmdLock);
            if (flag == 1) {
                TeleopCommands.requestControl(TeleopMedium::BLE);
            } else {
                TeleopCommands.releaseControl(TeleopMedium::BLE);
            }
            portEXIT_CRITICAL(&teleopCmdLock);
        }
    }
};

// ====================================================
// HIGH SPEED REMOTE DRIVING COMMAND PARSER
// ====================================================
class BleCommandCallbacks : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo& connInfo) override {
        NimBLEAttValue value = pCharacteristic->getValue();
        
        // We expect exactly 9 bytes: [Float X (4)] [Float Y (4)] [PID Bool (1)]
        if (value.length() == 9) {
            const uint8_t* payload = value.data();
            float rxX, rxY;

            // Extract offsets safely from data array positions
            memcpy(&rxX, &payload[0], sizeof(float));
            memcpy(&rxY, &payload[4], sizeof(float));
            bool rxPID = (payload[8] != 0);

            // Send command through the gateway. If BLE doesn't own the token, it is safely ignored!
            portENTER_CRITICAL(&teleopCmdLock);
            TeleopCommands.updateCommand(TeleopMedium::BLE, rxX, rxY, rxPID);
            portEXIT_CRITICAL(&teleopCmdLock);
        }
    }
};

// ====================================================
// BLE CALLBACKS: NETWORK STATE & FAILSAFES
// ====================================================
// 1. Corrected NimBLE 2.x Callbacks
class BleServerCallbacks : public NimBLEServerCallbacks {
    // Signature updated: ble_gap_conn_desc* replaced with NimBLEConnInfo&
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
        // Flip the BT LE connected Health Bit ON
        portENTER_CRITICAL(&globalDataBusLock);
        CurrentRobotData.health.hardwareBitmask |= Comms::HealthBit::BLE_CONNECTED;
        portEXIT_CRITICAL(&globalDataBusLock);

        // Restart the beacon so the Phone App can connect while Python is streaming!
        NimBLEDevice::startAdvertising();
    }

    // Signature updated: ble_gap_conn_desc* replaced with NimBLEConnInfo& + added int reason
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
        // HARD FAILSAFE: If link physically breaks, revoke token immediately!
        portENTER_CRITICAL(&teleopCmdLock);
        TeleopCommands.releaseControl(TeleopMedium::BLE);
        portEXIT_CRITICAL(&teleopCmdLock);
        
        // Do NOT print here! The heap is locked during callbacks! and Serial.print/ln is a blocking call
        // Serial.println("\n[BLE] Remote Control Disconnected. Failsafe triggered. Restarting advertising...");
        
        // Standard BLE protocol requires manually restarting advertising after a client drops
        NimBLEDevice::startAdvertising();

        // Flip the BT LE connected Health Bit OFF
        portENTER_CRITICAL(&globalDataBusLock);
        CurrentRobotData.health.hardwareBitmask &= ~Comms::HealthBit::BLE_CONNECTED;
        portEXIT_CRITICAL(&globalDataBusLock);
    }
};

void RadioManager::initRadios() {
    bool printLogs = (SysConfig.ACTIVE_DEBUG_MODE & SysConfig.DEBUG_USB);

    if (printLogs) {
        Serial.println("\n=== RADIO INFRASTRUCTURE BOOT ===");
    }

    // 1. BOOT WIFI
    if (SysConfig.WIFI_ACTIVE) {
        if (printLogs) {
            Serial.print("[WIFI] Connecting to: ");
            Serial.println(SysConfig.WIFI_SSID);
        }
        //WiFi.mode(WIFI_STA);
        WiFi.begin(SysConfig.WIFI_SSID.c_str(), SysConfig.WIFI_PASSWORD.c_str());
        
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
        if (printLogs) Serial.println("[WIFI] Disabled in SysConfig.");
    }

    // 2. BOOT BLUETOOTH
    if (SysConfig.BT_ACTIVE) {
        if (printLogs) {
            Serial.print("[BLUETOOTH] Advertising as: ");
            Serial.println(SysConfig.BT_NAME);
        }
        
        // 1. Initialize the NimBLE Hardware Radio
        NimBLEDevice::init(SysConfig.BT_NAME.c_str());

        // 🚨 CRITICAL FIX: Expand the BLE packet size limit!
        // The default is 23 bytes. Your Event struct is 35 bytes.
        // This prevents the hardware from shredding your packets.
        NimBLEDevice::setMTU(512);

        // --- ADD THESE LINES TO PRINT MAC ADDRESS ---
        if (printLogs) {
            Serial.print("[BLUETOOTH] Hardware BLE MAC Address: ");
            Serial.println(NimBLEDevice::getAddress().toString().c_str());
        }

        // FORCE COEXISTENCE POWER MANAGEMENT
        // WiFi.setSleep(true);

        // CRITICAL: Set the Bluetooth stack to be "polite"
        // This tells NimBLE to respect Wi-Fi's modem sleep windows
        // NimBLEDevice::setPower(ESP_PWR_LVL_P9);
        
        // 2. Create the Server and attach connection failsafes
        pServer = NimBLEDevice::createServer();
        pServer->setCallbacks(new BleServerCallbacks());

        // 3. Create the Custom Service
        NimBLEService *pService = pServer->createService(SERVICE_UUID);

        // ---------------------------------------------------------
        // THE HIGH-SPEED COMMAND & TELEMETRY PIPE (UUID: ffe1)
        // ---------------------------------------------------------
        pBleCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID,
            NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR | NIMBLE_PROPERTY::NOTIFY 
        );
        pBleCharacteristic->setCallbacks(new BleCommandCallbacks());

        // ---------------------------------------------------------
        // THE TOKEN AUTHORITY FLAG (UUID: ffe2)
        // ---------------------------------------------------------
        NimBLECharacteristic *pTokenChar = pService->createCharacteristic(
            TOKEN_CHAR_UUID,
            NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR 
        );
        pTokenChar->setCallbacks(new BleTokenCallbacks());
        // ---------------------------------------------------------
        
        // Start the Server (This automatically starts all attached services natively)
        pServer->start();

        // Configure advertising
        NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(SERVICE_UUID);
        pAdvertising->start();

        if (printLogs) Serial.println("[BLUETOOTH] GATT Server Online and Broadcasting.");
        
    } else {
         if (printLogs) Serial.println("[BLUETOOTH] Disabled in SysConfig.");
    }

    if (printLogs) {
        Serial.println("=================================\n");
    }
}

void RadioManager::connectWiFi(String ssid, String password) {
    bool printLogs = (SysConfig.ACTIVE_DEBUG_MODE & SysConfig.DEBUG_USB);
    if (ssid == "") return;
    
    // Disconnect if already connected before trying new credentials
    if (WiFi.status() == WL_CONNECTED) {
        disconnectWiFi();
    }

    if (printLogs) {
        Serial.print("\n[WIFI] CLI Requesting connection to: ");
        Serial.println(ssid);
        //Serial.println("[WIFI] Handshake running in background. Network Task is free!");
    }

    // MANDATORY: Explicitly ensure the stack is active before beginning
    WiFi.mode(WIFI_STA);
    
    // 1. Tell Core 0 to start the connection process
    WiFi.begin(ssid.c_str(), password.c_str());
    
    // 2. WE ARE DONE! NO WHILE LOOPS! NO DELAYS! 
    // Task_Network instantly goes back to listening to Foxglove and the CLI.
}



// void RadioManager::disconnectWiFi() {
//     WiFi.disconnect(true);
//     WiFi.mode(WIFI_OFF);
// }

bool RadioManager::isWiFiReady() {
    return SysConfig.WIFI_ACTIVE && (WiFi.status() == WL_CONNECTED);
}

void RadioManager::disconnectWiFi() {
    // 1. Soft disconnect: Drops the connection and erases credentials.
    WiFi.disconnect(true); 
    
    // 2. DO NOT USE WiFi.mode(WIFI_OFF)! 
    // We leave the mode in WIFI_STA so the TCP/IP stack stays alive 
    // and doesn't violently rip the sockets away from the WebSocketsServer!
    WiFi.mode(WIFI_STA); 
}

// BLUETOOTH (LE)
bool RadioManager::isBluetoothReady() {
    return SysConfig.BT_ACTIVE;
}

void RadioManager::connectBluetooth(String name) {
    // Implementation can use NimBLE setup configurations
}

// NEW: Status Check
bool RadioManager::isBleConnected() {
    return (pServer != nullptr && pServer->getConnectedCount() > 0);
}

// NEW: The Bluetooth LE Telemetry Blaster
void RadioManager::broadcastBLE(const uint8_t* data, size_t length) {
    // Only blast if the characteristic exists, the server is up, and a client is actively listening
    if (pBleCharacteristic != nullptr && pServer != nullptr && pServer->getConnectedCount() > 0) {
        pBleCharacteristic->setValue(data, length);
        pBleCharacteristic->notify(); // Blasts it to the Python Bridge!
    }
}

void RadioManager::disconnectBluetooth() {
    NimBLEDevice::deinit();
}