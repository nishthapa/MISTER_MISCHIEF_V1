#pragma once

class I_Barometer {
public:
    virtual bool init() = 0;
    virtual void update() = 0; // <--- NEW
    virtual float getPressurePa() = 0;
    virtual float getAltitudeCM() = 0;
    virtual float getTemperatureC() = 0;
        virtual ~I_Barometer() {}
};