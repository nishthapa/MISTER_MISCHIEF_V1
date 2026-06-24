#include "BMP280_Baro.h"
#include "config/I2CRegistry.h"
#include <math.h>

BMP280_Baro::BMP280_Baro() : basePressure(0.0f), initialized(false), 
                             baselineEstablished(false), baselineSampleCount(0), 
                             baselineAccumulator(0.0f), cachedPressurePa(0.0f), 
                             cachedAltitudeCM(0.0f), cachedTempC(0.0f) {}

bool BMP280_Baro::init() {
    if (!bmp.begin(I2CRegistry::I2C_ADDR_BMP280)) {
        return false;
    }

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     
                    Adafruit_BMP280::SAMPLING_X2,     
                    Adafruit_BMP280::SAMPLING_X16,    
                    Adafruit_BMP280::FILTER_X16,      
                    Adafruit_BMP280::STANDBY_MS_1); 

    // Instantly return to the RTOS scheduler. NO DELAYS!
    initialized = true;
    baselineEstablished = false;
    baselineSampleCount = 0;
    baselineAccumulator = 0.0f;
    return true;
}

// void BMP280_Baro::update() {
//     if (!initialized) return;
    
//     cachedTempC = bmp.readTemperature();
//     cachedPressurePa = bmp.readPressure();
    
//     // --- NON-BLOCKING IIR FILTER SETTLING ---
//     if (!baselineEstablished) {
//         baselineAccumulator += cachedPressurePa;
//         baselineSampleCount++;
        
//         // Wait for 16 samples to pass to guarantee the hardware filter has stabilized
//         if (baselineSampleCount >= 16) {
//             basePressure = baselineAccumulator / 16.0f;
//             baselineEstablished = true;
//         }
        
//         cachedAltitudeCM = 0.0f; // Force altitude to zero while calibrating
//         return; // Skip the math until calibration is done
//     }

//     // --- NORMAL OPERATION ---
//     if (basePressure > 0) {
//         cachedAltitudeCM = 44330.0f * (1.0f - pow(cachedPressurePa / basePressure, 0.1903f)) * 100.0f; // * 100 for cm output
//     }
// }

void BMP280_Baro::update() {
    if (!initialized) return;
    
    cachedTempC = bmp.readTemperature();
    cachedPressurePa = bmp.readPressure();
    
    // --- NON-BLOCKING BASELINE CALIBRATION ---
    if (!baselineEstablished) {
        // Accumulate samples over multiple ticks to let the hardware filter settle
        baselineAccumulator += cachedPressurePa;
        baselineSampleCount++;
        
        if (baselineSampleCount >= 30) { // Increased to 30 loops to allow real-world settling time
            basePressure = baselineAccumulator / 30.0f;
            baselineEstablished = true;
        }
        
        cachedAltitudeCM = 0.0f; 
        return; 
    }

    // --- NORMAL OPERATION ---
    if (basePressure > 0.0f) {
        // Compute altitude dynamically using your calibrated baseline pressure
        // Multiplying the final calculation by 100.0f yields the native output in centimeters
        float kelvin = cachedTempC + 273.15f;
        cachedAltitudeCM = (kelvin / 0.0065f) * (1.0f - powf(cachedPressurePa / basePressure, 0.190261f)) * 100.0f;
    }
}

float BMP280_Baro::getAltitudeCM() { return cachedAltitudeCM; }
float BMP280_Baro::getPressurePa() { return cachedPressurePa; }
float BMP280_Baro::getTemperatureC() { return cachedTempC; }