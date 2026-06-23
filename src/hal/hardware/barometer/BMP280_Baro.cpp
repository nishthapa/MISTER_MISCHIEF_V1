#include "BMP280_Baro.h"

BMP280_Baro::BMP280_Baro() : basePressure(1013.25f), initialized(false) {}

bool BMP280_Baro::init() {
    // 0x76 is the common address for the GY-91 clone BMP280.
    // If it fails, try 0x77.
    if (!bmp.begin(0x76)) {
        return false;
    }

    // Default settings for indoor navigation
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

    // Take a baseline reading at boot to act as "zero altitude"
    basePressure = bmp.readPressure(); // / 100.0f; // Convert Pa to hPa
    initialized = true;
    return true;
}

float BMP280_Baro::getAltitudeCM() {
    if (!initialized) return 0.0f;
    return bmp.readAltitude(basePressure) * 100.0f; // The library returns meters, multiply by 100 for CM
}

float BMP280_Baro::getPressurePa() {
    if (!initialized) return 0.0f;
    return bmp.readPressure(); // Returns raw Pascals directly
}

float BMP280_Baro::getTemperatureC() {
    if (!initialized) return 0.0f;
    return bmp.readTemperature(); // Returns Celsius
}