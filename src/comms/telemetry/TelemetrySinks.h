#pragma once

#include <Arduino.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include "ITelemetrySink.h"

// 1. The USB Serial Implementation
class USBSink : public ITelemetrySink {
public:
    void sendBinary(const uint8_t* buffer, size_t length) override {
        // Direct hardware write for maximum performance
        Serial.write(buffer, length);
    }
    
    bool isReady() override {
        // === THE ESP32-S3 NATIVE USB FIX ===
        // 1. static_cast<bool>(Serial) checks if the DTR line is high (Terminal is open)
        // 2. availableForWrite() checks the physical buffer. If it's less than our packet size, 
        //    the PC is disconnected or lagging, and we must abort to prevent a 1-second hard lock!
        return static_cast<bool>(Serial) && (Serial.availableForWrite() >= 64); 
    }
};

// 2. The Wi-Fi WebSocket Implementation
class WebSocketSink : public ITelemetrySink {
private:
    WebSocketsServer& ws;
    volatile int& clientCount;
    volatile unsigned long& lastConnectTime;

public:
    // We pass references so the sink can read the Logger's internal state!
    WebSocketSink(WebSocketsServer& webSocketRef, volatile int& activeClients, volatile unsigned long& connectTime) 
        : ws(webSocketRef), clientCount(activeClients), lastConnectTime(connectTime) {}

    void sendBinary(const uint8_t* buffer, size_t length) override {
        // We cast to (uint8_t*) to satisfy the WebSockets library's strict type requirements
        // Using broadcastBIN for zero-overhead binary streaming
        ws.broadcastBIN(buffer, length);
    }

    bool isReady() override {
        // The 1-second pause firewall is handled securely inside the sink!
        return (WiFi.status() == WL_CONNECTED && clientCount > 0 && (millis() - lastConnectTime > 1000));
    }
};

// 3. A Debug Sink that prints packets as Hexadecimal (Human readable, keeps architecture clean!)
class DebugHexSink : public ITelemetrySink {
public:
    void sendBinary(const uint8_t* buffer, size_t length) override {
        Serial.print("[TX BINARY] ");
        for (size_t i = 0; i < length; i++) {
            if (buffer[i] < 0x10) Serial.print("0");
            Serial.print(buffer[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    
    bool isReady() override { return true; }
};

/* // 4. The Bluetooth Implementation (Uncomment when BLE is ready!)
class BluetoothSink : public ITelemetrySink { ... }
*/