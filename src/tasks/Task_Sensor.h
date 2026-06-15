#pragma once

#include <Arduino.h>
#include "hal/interfaces/I_IMU.h"
#include "hal/interfaces/I_DistanceSensor.h"

// 1. Define the Context Struct
struct SensorContext {
    I_IMU* imu;
    I_DistanceSensor* sonar;
};

void SensorTask(void *pvParameters);