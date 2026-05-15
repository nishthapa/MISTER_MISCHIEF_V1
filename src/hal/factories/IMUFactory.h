#pragma once

#include "hal/interfaces/I_IMU.h"
#include "config/SensorConfig.h"
#include "config/PinConfig.h"

// Include all supported IMUs here
#include "hal/hardware/imu/MPU6050_IMU.h"

class IMUFactory {
public:
    // Returns a pointer to the generic interface based on the config!
    static I_IMU* createIMU() {
        
        if (IMUConfig::SELECTED_IMU == IMUConfig::IMU_MPU6050) {
            // Allocate the hardware statically so it survives in memory
            static MPU6050_IMU mpu(HardwarePins::PIN_I2C_SDA, HardwarePins::PIN_I2C_SCL, HardwarePins::PIN_IMU_INT, 0x68);
            return &mpu;
        }

        else if (IMUConfig::SELECTED_IMU == IMUConfig::IMU_BNO055) {
            // Placeholder for your future BNO055Sensor!
            // static BNO055_IMU bno(HardwarePins::PIN_I2C_SDA, HardwarePins::PIN_I2C_SCL, HardwarePins::PIN_IMU_INT);
            // return &bno;
        }

        // If IMU_NONE or unknown is selected
        return nullptr;
    }
};