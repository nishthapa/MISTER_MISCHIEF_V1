#pragma once

namespace FactoryDefaults {

    // ==========================================
    // SERIAL COMMS BAUD RATE
    // ==========================================
    constexpr unsigned long SERIAL_BAUD_RATE = 115200;

    // ==========================================
    // PERSONALITY SETTINGS
    // ==========================================
    // Range: 10.0 to 100.0 (Base movement speed)
    constexpr float CRUISING_SPEED = 40.0f;
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
    // OBSTACLE AVOIDANCE ESCAPE SEQUENCE TUNING
    // ==========================================
    // PART 1: The Radar Sweep
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
    
    // PART 2: The Escape Maneuver (Backing Up, Spinning, and Driving Forward)
    // RANGE: 30.0f TO 100.0f // The baseline motor speed used during the escape maneuvers
    constexpr float OBSTACLE_ESCAPE_BASE_SPEED = 30.0f;
    // RANGE: 500 TO 2000 // How long he blindly backs up after hitting a wall
    constexpr unsigned long OBSTACLE_BACKUP_DURATION_MS = 1000;
    // RANGE: 90.0f TO 360.0f // How far the robot physically turns while scanning
    constexpr float OBSTACLE_SWEEP_ANGLE_DEG = 160.0f;
    // RANGE: 2000 TO 5000 // Max time allowed to spin tracks in the air during radar sweep
    constexpr unsigned long OBSTACLE_SWEEP_TIMEOUT_MS = 3000;
    // RANGE: 2000 TO 6000 // Max time allowed to try aligning to the escape heading (Infinite Loop Fix)
    constexpr unsigned long OBSTACLE_ALIGN_TIMEOUT_MS = 4000;
    // RANGE: 2.0f TO 10.0f // How accurate the alignment must be before driving forward
    constexpr float OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG = 5.0f;
    // RANGE: 500 TO 2000 // How long he drives forward to escape the obstacle
    constexpr unsigned long OBSTACLE_ESCAPE_DURATION_MS = 1000;

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
    // RANGE: 5000 TO 20000  // Time the robot spends wandering blindly after a violent shake
    constexpr unsigned long DIZZY_DURATION_MS = 10000;
    
    
    // RANGE: 1000 TO 5000   // Wait time in hands before engaging Compass Lock (filters out active play)
    constexpr unsigned long COMPASS_LOCK_ENTRY_SETTLE_MS = 3000;
    
    // RANGE: 1000 TO 5000   // Wait time on a flat surface before disengaging Compass Lock
    constexpr unsigned long COMPASS_LOCK_EXIT_SETTLE_MS = 3000;



    // --- Handling & Lift Detection (The Pickup Sensitivity) ---
    // RANGE: 15.0f TO 35.0f // Degrees of tilt required to definitively say "a human grabbed me"
    constexpr float TILT_HANDLING_THRESHOLD = 22.0f;      
    
    // RANGE: 1.05f TO 1.30f // Earth is 1.0G. A gentle lift is ~1.1G. A fast yank is >1.3G.
    constexpr float GFORCE_LIFT_UP_THRESHOLD = 1.15f;     
    
    // RANGE: 0.70f TO 0.95f // Earth is 1.0G. A gentle lowering is ~0.85G. A fast drop is <0.7G.
    constexpr float GFORCE_LIFT_DOWN_THRESHOLD = 0.85f;   
    
    // RANGE: 100.0f TO 400.0f // Rotational energy spike to act as a fallback if linear G-force misses the pickup
    constexpr float LIFT_ENERGY_SPIKE_THRESHOLD = 200.0f; 

    // --- Floor & Table Detection ---
    // RANGE: 2.0f TO 10.0f  // Degrees of slop allowed for imperfect chassis builds or slightly slanted tables
    constexpr float UPRIGHT_ANGLE_TOLERANCE = 5.0f;       
    
    // RANGE: 35.0f TO 60.0f // MPU6050 sensor noise averages 30-45. A steady human hand averages 80-150.
    constexpr float PERFECTLY_STILL_ENERGY = 45.0f;       
    
    // RANGE: 80.0f TO 200.0f // Max rotational energy allowed to be considered "held steady" on a book/hand
    constexpr float STEADY_HOLD_ENERGY_MAX = 150.0f;      

    // --- The Dizzy Bar ---
    // RANGE: 15.0f TO 50.0f // Baseline micro-movements (walking, normal carrying) that generate ZERO dizzy charge
    constexpr float DIZZY_ENERGY_DEADBAND = 30.0f;        
    
    // RANGE: 0.001f TO 0.01f // Defines how fast the dizzy bar fills. At 100Hz, 0.002 requires ~4s of shaking.
    constexpr float DIZZY_CHARGE_RATE = 0.002f;           
    
    // RANGE: 0.990f TO 0.999f // MUST equal (1.0f - DIZZY_CHARGE_RATE). How fast the bar drains when still.
    constexpr float DIZZY_DECAY_RATE = 0.998f;
    
    // RANGE: 100.0f TO 300.0f // The "Full" capacity of the Dizzy Bar required to trigger the mode
    constexpr float DIZZY_TRIGGER_THRESHOLD = 150.0f;     

    // --- Energy Smoothing (Exponential Moving Average) ---
    // RANGE: 0.01f TO 0.25f // 10% trust in the new reading. Higher catches tremors faster, lower smooths more.
    constexpr float ENERGY_EMA_ALPHA = 0.10f;             
    
    // RANGE: 0.75f TO 0.99f // MUST equal (1.0f - ENERGY_EMA_ALPHA). The trust in the historical average.
    constexpr float ENERGY_EMA_BETA = 0.90f;
    
    // How fast frustration cools down when left alone
    constexpr float FRUSTRATION_COOLDOWN_RATE = 0.5f; 
    constexpr float FRUSTRATION_HEATUP_RATE = 1.0f;
    
    // Ticks required to trigger ANGRY mood (1 tick = 10ms in the control loop)
    constexpr float DISTANCE_HOLD_FRUSTRATION_LIMIT = 300.0f;

    // ==========================================
    // SENSOR TUNING
    // ==========================================  
    //constexpr float IMU_MADGWICK_BETA = 0.05f; 
    // Range: 0.01 to 0.10 (Rad/s to ignore mechanical chassis vibration)
    constexpr float IMU_GYRO_DEADBAND = 0.02f; 
    // Range: 50.0 to 400.0 (Max trusted range of HC-SR04)
    constexpr float SONAR_MAX_DIST = 200.0f;

    // --- IMU ORIENTATION ---
    static const bool IMU_INVERT_ROLL = false;
    static const bool IMU_INVERT_PITCH = true;  // Fixed the physical mount
    static const bool IMU_INVERT_YAW = true;    // Fixed the runaway spin

    // ==========================================
    // MADGWICK FILTER TUNING
    // ==========================================
    // Range: 0.01 to 0.5 (Lower = less drift but slower response)
    constexpr float MADGWICK_FILTER_BETA = 0.04f; // governs how much to trust the accelerometer // Lowered to kill the twitching!

    // ==========================================
    // AUTOTUNING
    // ==========================================
    // Range: 1000 to 10000 (Milliseconds to wait before violently starting Autotune)
    constexpr unsigned long AUTOTUNE_START_DELAY_MS = 5000;
    // Range: 2000 to 10000 (Max ms to wait for a 5-degree turn before assuming broken hardware)
    constexpr unsigned long AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS = 5000;

    // ==========================================
    // UNIFIED POINT TURN PID TUNING (Stationary)
    // ==========================================
    constexpr float PID_POINT_P = 2.0f;
    constexpr float PID_POINT_I = 0.0f;
    constexpr float PID_POINT_D = 0.5f;
    constexpr float PID_POINT_LIM = 120.0f;
    constexpr float PID_POINT_ILIM = 0.0f;
    constexpr float PID_POINT_DEAD = 3.0f; // Degrees

    // ==========================================
    // UNIFIED ARC TURN PID TUNING (Rolling)
    // ==========================================
    constexpr float PID_ARC_P = 1.5f;
    constexpr float PID_ARC_I = 0.0f;
    constexpr float PID_ARC_D = 0.2f;
    constexpr float PID_ARC_LIM = 100.0f;
    constexpr float PID_ARC_ILIM = 0.0f;
    constexpr float PID_ARC_DEAD = 2.0f; // Degrees

/*
    // ==========================================
    // HEADING HOLD PID TUNING
    // ==========================================
    constexpr float PID_HEADING_P = 1.5f;   // Range: 0.1 to 10.0
    constexpr float PID_HEADING_I = 0.0f;   // Range: 0.0 to 2.0
    constexpr float PID_HEADING_D = 0.2f;   // Range: 0.0 to 5.0
    constexpr float PID_HEADING_LIM = 100.0f; // Range: 10.0 to 255.0
    constexpr float PID_HEADING_ILIM = 0.0f;  // Range: 0.0 to 255.0
    constexpr float PID_HEADING_DEAD = 2.0f;  // Range: 0.5 to 10.0 (Degrees)
*/

/*
    // ==========================================
    // COMPASS LOCK PID TUNING
    // ==========================================
    constexpr float PID_COMPASS_P = 2.0f;
    constexpr float PID_COMPASS_I = 0.0f;
    constexpr float PID_COMPASS_D = 0.5f;
    constexpr float PID_COMPASS_LIM = 120.0f;
    constexpr float PID_COMPASS_ILIM = 0.0f;
    constexpr float PID_COMPASS_DEAD = 3.0f;
*/

    // ==========================================
    // DISTANCE HOLD PID TUNING
    // ==========================================
    constexpr float PID_DIST_P = 2.5f;
    constexpr float PID_DIST_I = 0.1f;
    constexpr float PID_DIST_D = 0.5f;
    constexpr float PID_DIST_LIM = 80.0f;
    constexpr float PID_DIST_ILIM = 20.0f;
    constexpr float PID_DIST_DEAD = 2.0f; // Centimeters


/*    
    // ==========================================
    // OBSTACLE AVOIDANCE PATH SCAN PID TUNING
    // ==========================================
    constexpr float PID_OBSTACLE_P = 1.2f;
    constexpr float PID_OBSTACLE_I = 0.0f;
    constexpr float PID_OBSTACLE_D = 0.1f;
    constexpr float PID_OBSTACLE_LIM = 90.0f;
    constexpr float PID_OBSTACLE_ILIM = 0.0f;
    constexpr float PID_OBSTACLE_DEAD = 2.0f;
*/


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


    // 1. Define the Bitmask Values (Powers of 2)
    constexpr uint8_t DEBUG_ACTIVE       = 0;       // 00000000
    constexpr uint8_t DEBUG_USB       = 1 << 0;  // 00000001 (Decimal 1)
    constexpr uint8_t DEBUG_WIFI      = 1 << 1;  // 00000010 (Decimal 2)
    constexpr uint8_t DEBUG_BLUETOOTH = 1 << 2;  // 00000100 (Decimal 4)

    // ========================================================
    // 2. THE MASTER DEBUG SWITCH
    // Combine modes using the '|' (OR) operator.
    // Example: DEBUG_USB | DEBUG_WIFI
    // To disable everything, just use: DEBUG_OFF
    // ========================================================
    constexpr uint8_t ACTIVE_DEBUG_MODE = DEBUG_USB | DEBUG_WIFI | DEBUG_BLUETOOTH; // Auto selects the enabled debug outputs based on active comms.
}