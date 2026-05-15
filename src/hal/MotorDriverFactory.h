#pragma once

#include "hal/I_MotorDriver.h"
#include "config/MotorDriverConfig.h"
#include "config/PinConfig.h"
#include "hal/XY160D_MotorDriver.h"

class MotorDriverFactory {
public:
    static I_MotorDriver* createMotorDriver() {
        if (MotorDriverConfig::SELECTED_DRIVER == MotorDriverConfig::DRIVER_XY160D) {
            
            // Allocate statically so it survives in memory forever
            static XY160D_MotorDriver driver(
                HardwarePins::PIN_MOTOR_LEFT_FWD,
                HardwarePins::PIN_MOTOR_LEFT_REV,
                HardwarePins::PIN_MOTOR_RIGHT_FWD,
                HardwarePins::PIN_MOTOR_RIGHT_REV
            );
            return &driver;
        }

        return nullptr;
    }
};