#pragma once
#include <Arduino.h>

namespace SystemConfig {
    // ==========================================
    // OS, MEMORY, & TASK SCHEDULING
    // ==========================================

    // --- FreeRTOS Task Configuration ---
    constexpr uint32_t TASK_STACK_SIZE = 8192;       // Memory allocated per core (in words)

    constexpr UBaseType_t SENSOR_TASK_PRIORITY = 1;  // 0 Priority for sensor reading and telemetry      
    constexpr UBaseType_t CONTROL_LOOP_TASK_PRIORITY = 1; // 1 Priority for main control loop and decision making
    constexpr UBaseType_t TELEMETRY_TASK_PRIORITY = 1; // 1 Priority for telemetry and networking

    // SWAP FOR WIFI NETWORKING STABILITY
    constexpr UBaseType_t SENSOR_TASK_CORE_AFFINITY = 1;  // <--- Networking & Sonar belongs on APP CPU!
    constexpr UBaseType_t CONTROL_LOOP_TASK_CORE_AFFINITY = 1; // <--- Physics & Math runs on PRO CPU!
    constexpr UBaseType_t TELEMETRY_TASK_CORE_AFFINITY = 0; // <--- Keep the Websocket server on PRO CPU for stability!
    
    // --- Loop Rates ---
    constexpr unsigned long MAIN_LOOP_TICK_RATE_MS = 10;  // 10ms = 100Hz (The Physics Engine metronome)
    constexpr unsigned long TELEMETRY_PING_DELAY_MS = 80; // 80ms = 12.5Hz (How fast the Mouth talks)

    // ==========================================
    // BOOT SEQUENCE & HARDWARE TIMINGS
    // ==========================================
    
    // RANGE: 3000 TO 10000 // Time to wait for Telnet to connect before booting the robot
    constexpr unsigned long TELNET_WAIT_TIME_MS = 8000;   
    
    // RANGE: 100 TO 1000   // Time to let the power bus stabilize before interrogating I2C
    constexpr unsigned long HARDWARE_WAKE_DELAY_MS = 500; 

    // --- IMU Boot Watchdog ---
    constexpr int IMU_MAX_RETRIES = 5;                    // How many times to try booting the MPU6050
    constexpr unsigned long IMU_RETRY_DELAY_MS = 1000;    // Wait time between failures

    constexpr int TELNET_PORT = 23; // Moved from the old WiFiConfig!

    constexpr int WEBSOCKET_PORT = 81;
}