#pragma once

#include <Arduino.h>

class ITelemetrySink {
public:
    virtual ~ITelemetrySink() = default;

    // OLD: virtual void sendTelemetry(...)
    // NEW: Accepts a pointer to raw bytes and the total length of the packet
    virtual void sendBinary(const uint8_t* buffer, size_t length) = 0;
    
    // Keeps the hardware-level health check (e.g., is Bluetooth paired?)
    virtual bool isReady() = 0; 
};