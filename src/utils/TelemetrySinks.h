// --- src/utils/TelemetrySinks.h ---
#pragma once

#include <Arduino.h>
#include <WebSocketsServer.h>
#include <WiFi.h>

// 1. The Generic Interface
class ITelemetrySink {
public:
    virtual ~ITelemetrySink() = default;
    virtual void transmit(const char* jsonString) = 0;
    virtual bool isReady() = 0; 
};

// 2. The USB Serial Implementation
class USBSink : public ITelemetrySink {
public:
    void transmit(const char* jsonString) override {
        Serial.println(jsonString);
    }
    bool isReady() override {
        return static_cast<bool>(Serial); 
    }
};

// 3. The Wi-Fi WebSocket Implementation
class WebSocketSink : public ITelemetrySink {
private:
    WebSocketsServer& ws;
    volatile int& clientCount;
    volatile unsigned long& lastConnectTime;

public:
    // We pass references so the sink can read the Logger's internal state!
    WebSocketSink(WebSocketsServer& webSocketRef, volatile int& activeClients, volatile unsigned long& connectTime) 
        : ws(webSocketRef), clientCount(activeClients), lastConnectTime(connectTime) {}

    void transmit(const char* jsonString) override {
        ws.broadcastTXT(jsonString);
    }

    bool isReady() override {
        // The 1-second pause firewall is handled securely inside the sink!
        return (WiFi.status() == WL_CONNECTED && clientCount > 0 && (millis() - lastConnectTime > 1000));
    }
};

/* // 4. The Bluetooth Implementation (Uncomment when BLE is ready!)
class BluetoothSink : public ITelemetrySink { ... }
*/