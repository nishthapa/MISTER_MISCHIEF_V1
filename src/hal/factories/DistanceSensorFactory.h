#pragma once
#include "hal/interfaces/I_DistanceSensor.h"
#include "config/SensorConfig.h"
#include "config/PinConfig.h"
#include "hal/hardware/distancesensor/HCSR04_Sonar.h"

// #include "hal/VL53L0X_Laser.h" // You will uncomment this when you build the ToF driver!

class DistanceSensorFactory {
public:
    static I_DistanceSensor* createDistanceSensor() {
        
        if (DistanceSensorConfig::SELECTED_DIST_SENSOR == DistanceSensorConfig::SENSOR_HCSR04) {
            // Allocate the Ultrasonic sensor statically
            static HCSR04_Sonar sonar(HardwarePins::PIN_SONAR_TRIG, HardwarePins::PIN_SONAR_ECHO);
            return &sonar;
        }
        else if (DistanceSensorConfig::SELECTED_DIST_SENSOR == DistanceSensorConfig::SENSOR_VL53L0X) {
            // Placeholder for your future Time-of-Flight Laser Sensor!
            // static VL53L0X_Laser laser(HardwarePins::PIN_I2C_SDA, HardwarePins::PIN_I2C_SCL);
            // return &laser;
        }

        return nullptr;
    }
};