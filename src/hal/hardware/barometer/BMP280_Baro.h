#pragma once

#include "hal/interfaces/I_Barometer.h"
#include <Adafruit_BMP280.h>

class BMP280_Baro : public I_Barometer {
private:
    Adafruit_BMP280 bmp;
    float basePressure; // Baseline pressure taken at boot
    bool initialized;

public:
    BMP280_Baro();
    bool init() override;
    float getPressurePa() override;
    float getAltitudeCM() override;
    float getTemperatureC() override;
};