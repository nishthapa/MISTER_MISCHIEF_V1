#pragma once

class I_MotorDriver {
public:
    // The Contract: Every motor driver in the world must be able to do these 3 things!
    //virtual void init() = 0;
    virtual bool init() = 0;
    
    // Accepts speeds from -100.0f (Full Reverse) to 100.0f (Full Forward)
    virtual void drive(float leftSpeed, float rightSpeed) = 0;
    
    virtual void stop() = 0;

    // Virtual destructor for safe memory cleanup
    virtual ~I_MotorDriver() {}
};