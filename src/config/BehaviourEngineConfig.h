#pragma once

namespace BehaviourConfig {
    // ==========================================
    // MISTER MISCHIEF - PHYSICS & BEHAVIOUR TUNING
    // ==========================================

    // --- State Timers (in milliseconds) ---
    // RANGE: 5000 TO 20000  // Time the robot spends wandering blindly after a violent shake
    constexpr unsigned long DIZZY_DURATION_MS = 10000;
    
    // RANGE: 1000 TO 5000   // Wait time in hands before engaging Compass Lock (filters out active play)
    constexpr unsigned long COMPASS_ENTRY_SETTLE_MS = 3000;
    
    // RANGE: 1000 TO 5000   // Wait time on a flat surface before disengaging Compass Lock
    constexpr unsigned long COMPASS_EXIT_SETTLE_MS = 3000;

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
}