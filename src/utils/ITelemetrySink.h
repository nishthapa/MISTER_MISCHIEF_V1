#pragma once

class ITelemetrySink {
public:
    virtual ~ITelemetrySink() = default;
    
    // The contract every medium must follow
    virtual void transmit(const char* jsonString) = 0;
    virtual bool isReady() = 0; 
};