#pragma once

#include "hal/interfaces/I_Barometer.h"
#include "config/SensorConfig.h" // Assumes BarometerConfig::SELECTED_BAROMETER is defined here
#include "hal/hardware/barometer/BMP280_Baro.h"

class BarometerFactory {
public:
    static I_Barometer* createBaro() {
        // Assuming your SensorConfig.h has a BarometerConfig namespace or enum
        if (BarometerConfig::SELECTED_BAROMETER == BarometerConfig::BARO_BMP280) {
            // Correct syntax: no parentheses for a zero-argument constructor
            static BMP280_Baro bmp; 
            return &bmp;
        }
        return nullptr;
    }
};