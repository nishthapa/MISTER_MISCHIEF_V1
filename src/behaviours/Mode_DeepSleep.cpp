#include "behaviours/Mode_DeepSleep.h"
#include "config/PinConfig.h" // Needed to know which pin the IMU uses
#include <Arduino.h>
#include "utils/RemoteLogger.h" // <-- ADD THIS INCLUDE
#include <esp_sleep.h> // For the ESP sleep commands

Mode_DeepSleep::Mode_DeepSleep(KinematicsEngine* k) {
    kinematics = k;
}

void Mode_DeepSleep::onEnter(const volatile GlobalSensorState& sensorState) {
    logger.println("Entering Ultra Low Power Mode...");
    kinematics->stop(); // Force stop before sleeping

    // 1. Tell the ESP32 to wake up if the IMU INT pin goes HIGH (someone tapped the robot)
    // You will later configure the MPU6050's Wake-on-Motion register in the HAL to trigger this pin
    esp_sleep_enable_ext0_wakeup((gpio_num_t)HardwarePins::PIN_IMU_INT, 1);
    
    // 2. You can also add a timer wakeup if you want it to check for your phone via BLE every 10 mins
    // esp_sleep_enable_timer_wakeup(600000000); // 10 minutes in microseconds

    // 3. Goodnight. (The code physically stops executing here until a reset)
    esp_deep_sleep_start(); 
}

void Mode_DeepSleep::update(const RobotMood& currentMood, const volatile GlobalSensorState& sensorState) {
    // Never runs, because onEnter() kills the CPU!
}