#pragma once

#include "hal/interfaces/I_Barometer.h"
#include <Adafruit_BMP280.h>

class BMP280_Baro : public I_Barometer {
private:
    Adafruit_BMP280 bmp;
    float basePressure; // Baseline pressure taken at boot
    bool initialized;

    // --- Non-blocking calibration state ---
    bool baselineEstablished;
    uint8_t baselineSampleCount;
    float baselineAccumulator;

    // --- Cache variables ---
    float cachedPressurePa;
    float cachedAltitudeCM;
    float cachedTempC;

public:
    BMP280_Baro();
    bool init() override;
    void update() override; // <--- NEW
    float getPressurePa() override;
    float getAltitudeCM() override;
    float getTemperatureC() override;
};