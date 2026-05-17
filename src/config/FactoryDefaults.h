#pragma once

namespace FactoryDefaults {

    // ==========================================
    // PERSONALITY SETTINGS
    // ==========================================
    // Range: 10.0 to 100.0 (Base movement speed)
    constexpr float CRUISING_SPEED = 30.0f;
    // Range: 10.0 to 150.0 (When to start worrying about a wall)
    constexpr float OBSTACLE_TRIGGER_CM = 30.0f;
    // Range: 5.0 to 50.0 (How far to stay from the target in follow mode)
    constexpr float MAINTAIN_DISTANCE_CM = 15.0f;

    // ==========================================
    // MOTOR TUNING
    // ==========================================
    // Range: 0 to 255 (Absolute lowest PWM that overcomes gear friction)
    constexpr int MOTOR_MIN_PWM = 60; 

    // ==========================================
    // OBSTACLE AVOIDANCE TUNING
    // ==========================================
    // Range: 10.0 to 90.0 (Degrees to turn head left/right)
    constexpr float OBS_SWEEP_ANGLE = 45.0f;   
    // Range: 30.0 to 150.0 (Rotation speed during scan)
    constexpr float OBS_SWEEP_SPEED = 90.0f; 
    // Range: 50 to 1000 (Milliseconds to let sonar sound waves settle)
    constexpr int   OBS_SWEEP_PAUSE = 200;  
    // Range: 20.0 to 200.0 (Distance required to declare a path "clear")
    constexpr float OBS_CLEAR_THRESH = 40.0f; 
    // Range: 1.0 to 15.0 (Buffer to prevent rapid left/right jitter)
    constexpr float OBS_HYSTERESIS = 5.0f;          

    // ==========================================
    // BEHAVIOURAL TIMERS AND LIMITS
    // ==========================================
    // Range: 50 to 255
    constexpr int   DIZZY_SPIN_PWM = 100;
    // Range: 500 to 10000 (Milliseconds to spin)
    constexpr int   DIZZY_SPIN_TIME = 2000;  
    // Range: 1000 to 60000 (Cooldown between spins)
    constexpr int   DIZZY_COOLDOWN = 5000;   
    // Range: 1000 to 300000 (Auto-sleep if no movement)
    constexpr int   SLEEP_TIMEOUT_MS = 10000; 
    // Range: 1.1 to 2.5 (G-Force threshold to wake up on pickup)
    constexpr float SLEEP_WAKE_G = 1.2f;            

    // ==========================================
    // SENSOR TUNING
    // ==========================================
    // Range: 0.01 to 0.5 (Lower = less drift but slower response)
    constexpr float IMU_MADGWICK_BETA = 0.05f; 
    // Range: 0.01 to 0.10 (Rad/s to ignore mechanical chassis vibration)
    constexpr float IMU_GYRO_DEADBAND = 0.02f; 
    // Range: 50.0 to 400.0 (Max trusted range of HC-SR04)
    constexpr float SONAR_MAX_DIST = 200.0f;  

    // ==========================================
    // HEADING HOLD PID TUNING
    // ==========================================
    constexpr float PID_HEADING_P = 1.5f;   // Range: 0.1 to 10.0
    constexpr float PID_HEADING_I = 0.0f;   // Range: 0.0 to 2.0
    constexpr float PID_HEADING_D = 0.2f;   // Range: 0.0 to 5.0
    constexpr float PID_HEADING_LIM = 100.0f; // Range: 10.0 to 255.0
    constexpr float PID_HEADING_ILIM = 0.0f;  // Range: 0.0 to 255.0
    constexpr float PID_HEADING_DEAD = 2.0f;  // Range: 0.5 to 10.0 (Degrees)

    // ==========================================
    // COMPASS LOCK PID TUNING
    // ==========================================
    constexpr float PID_COMPASS_P = 2.0f;
    constexpr float PID_COMPASS_I = 0.0f;
    constexpr float PID_COMPASS_D = 0.5f;
    constexpr float PID_COMPASS_LIM = 120.0f;
    constexpr float PID_COMPASS_ILIM = 0.0f;
    constexpr float PID_COMPASS_DEAD = 3.0f;

    // ==========================================
    // DISTANCE HOLD PID TUNING
    // ==========================================
    constexpr float PID_DIST_P = 2.5f;
    constexpr float PID_DIST_I = 0.1f;
    constexpr float PID_DIST_D = 0.5f;
    constexpr float PID_DIST_LIM = 80.0f;
    constexpr float PID_DIST_ILIM = 20.0f;
    constexpr float PID_DIST_DEAD = 2.0f; // Centimeters

    // ==========================================
    // OBSTACLE AVOIDANCE PATH SCAN PID TUNING
    // ==========================================
    constexpr float PID_OBSTACLE_P = 1.2f;
    constexpr float PID_OBSTACLE_I = 0.0f;
    constexpr float PID_OBSTACLE_D = 0.1f;
    constexpr float PID_OBSTACLE_LIM = 90.0f;
    constexpr float PID_OBSTACLE_ILIM = 0.0f;
    constexpr float PID_OBSTACLE_DEAD = 2.0f;

    // ==========================================
    // SYSTEM STATE
    // ==========================================
    constexpr bool BRAIN_ACTIVE = true;

    // ==========================================
    // NETWORK AND COMMS
    // ==========================================
    constexpr const char* WIFI_SSID = "";       
    constexpr const char* WIFI_PASSWORD = "";
    constexpr bool WIFI_ACTIVE = false;         
    constexpr const char* BT_NAME = "Mister_Mischief";
    constexpr bool BT_ACTIVE = false;

    // ==========================================
    // DEBUG AND TELEMETRY
    // ==========================================
    constexpr bool SERIAL_DEBUG_MASTER = true;
    constexpr bool SERIAL_DEBUG_IMU = true;
    constexpr bool SERIAL_DEBUG_SONAR = true;
    constexpr bool SERIAL_DEBUG_MOTOR_DRIVER = true;
}