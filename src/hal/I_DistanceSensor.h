#pragma once

class I_DistanceSensor {
public:
    virtual void init() = 0;
    
    // Returns distance in centimeters. Returns -1.0f if out of range/error.
    virtual float getDistanceCM() = 0;
    
    virtual ~I_DistanceSensor() {}
};