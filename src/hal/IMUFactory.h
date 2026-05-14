#pragma once

#include "hal/I_IMU.h"
#include "config/IMUConfig.h"
#include "config/PinConfig.h"

// Include all supported IMUs here
#include "hal/MPU6050_IMU.h"

class IMUFactory {
public:
    // Returns a pointer to the generic interface based on the config!
    static I_IMU* createIMU() {
        
        if (IMUConfig::SELECTED_IMU == IMUConfig::IMU_MPU6050) {
            // Allocate the hardware statically so it survives in memory
            static MPU6050_IMU mpu(HardwarePins::PIN_I2C_SDA, HardwarePins::PIN_I2C_SCL, HardwarePins::PIN_IMU_INT, 0x68);
            return &mpu;
        }

        // If IMU_NONE or unknown is selected
        return nullptr;
    }
};