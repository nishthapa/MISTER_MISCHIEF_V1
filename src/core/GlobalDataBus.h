#pragma once

#include <Arduino.h> // <--- for the FreeRTOS definitions
#include "core/RobotState.h"       // For SystemMode enum
#include "behaviours/RobotMood.h"  // For RobotMood enum
#include "hal/interfaces/I_IMU.h"
#include <stdint.h>
#include "config/PacketIDRegistry.h" // <-- Brings in your Comms::HealthBit definitions!

// =========================================================================
// 1. STATE DATA (The Absolute Truth of the Robot)
// =========================================================================
struct PhysicsState {
    FusedAngles imuAngles = {0.0f, 0.0f, 0.0f, 0.0f, false, 0}; // Safe zeroes      // MsgId 101: Attitude & G-Force
    int16_t leftMotorPWM = 0;       // MsgId 104: Actuators (Left Track Current Power)
    int16_t rightMotorPWM = 0;      // MsgId 104: Actuators (Right Track Current Power)
};

struct SensorState {
    float distanceCM = -1.0f;           // MsgId 110: Sonar Distance
    uint16_t batteryVoltageMV = 0;  // MsgId 111: Future INA226 Voltage Expansion
    int16_t currentDrawMA = 0;      // MsgId 111: Future INA226 Current Expansion
};

struct SystemHealthState {
    uint32_t loopTimeUs = 0;        // MsgId 130: Main physics loop speed
    uint32_t freeHeap = 0;          // MsgId 130: RAM available
    uint16_t hardwareBitmask = 0;   // MsgId 130: Alive components (Uses Comms::HealthBit)
    int8_t wifiRSSI = 0;            // MsgId 132: Wi-Fi Signal Strength
    int8_t bleRSSI = 0;             // MsgId 132: Bluetooth Signal Strength
};

// CONTROL DEBUG (PID & Target tracking)
struct ControlDebugState {
    float targetHeading = 0.0f; 
    float headingError = 0.0f;  
};

// NEW: Unified Event State
struct EventState {
    // 1. Semantic Events (The final triggers)
    bool hazardDetected = false;       
    bool teaseConfirmed = false;       
    bool targetVanished = false;       
    bool dizzyTriggered = false;       
    bool dizzyFinished = false;        
    bool readyForCompassLock = false;  
    bool safelyLanded = false;         
    bool frustrationPeaked = false;    

    // 2. Internal Metrics & States (Continuous variables from the handler)
    float dizzyBarYaw = 0.0f;
    float dizzyBarPitch = 0.0f;
    float dizzyBarRoll = 0.0f;
    float smoothedTotalEnergy = 0.0f;
    float frustrationLevel = 0.0f;
    
    // 3. Internal Latches
    bool isHandTeasing = false;
    bool isHandVanishing = false;
    bool isHandling = false;
    bool hasExperiencedLift = false;
    bool isLowering = false;
    bool hasLanded = false;
    bool isDizzy = false;
};

// FOR TELEMETRY (Intermediate Math & Tuning Metrics)
struct PerceptionMetrics {
    float distanceDelta = 0.0f;
    float totalRawEnergy = 0.0f;
    float rawYawEnergy = 0.0f;
    float rawPitchEnergy = 0.0f;
    float rawRollEnergy = 0.0f;
    float currentGForce = 0.0f;
};

// COGNITIVE STATE (MODE & MOOD)
struct CognitiveState {
    SystemMode systemMode = SystemMode::BOOTING;
    RobotMood robotMood = Moods::HAPPY; // Default mood
};

// The Master Wrapper
struct GlobalDataBank {
    PhysicsState physics;
    SensorState sensors;
    SystemHealthState health;
    EventState events; // <-- New centralized semantic events
    PerceptionMetrics perception; // <-- The new tuning mirror in the telemetry
    CognitiveState cognition; // <-- Mode and Mood
    ControlDebugState controlDebug; // <-- For PID Target and error
};

// =========================================================================
// 2. INBOUND COMMANDS (Internal Routing)
// =========================================================================
struct HardwareCommandBus {
    bool requestGyroCalibration = false;
    bool requestAccelCalibration = false;
    bool requestMagCalibration = false;
    float targetFilterBeta = 0.0f;
    bool requestMotorTest = false;
    uint8_t diagnosticAnswer = 0; // <--- 0 = waiting, 1-5 = user answer
    bool requestAutotune = false; // <--- NEW: Flag for Autotune
};

struct TeleopCommandBus {
    float joyX = 0.0f;       // -1.0f (Left) to 1.0f (Right)
    float joyY = 0.0f;       // -1.0f (Reverse) to 1.0f (Forward)
    bool usePIDDrive = false; // Toggles between RAW motor mixing and PID Heading Hold
    bool isConnected = false; // Failsafe: Stops robot if BLE drops
};

// The global lock object
// extern portMUX_TYPE globalDataBusLock; // TODO: Flesh it out for atomic reads/writes

// =========================================================================
// GLOBAL CROSS-CORE INSTANCES
// =========================================================================
// extern volatile GlobalDataBank CurrentRobotData;
// extern volatile HardwareCommandBus HardwareCommands;
// extern volatile TeleopCommandBus TeleopCommands;

// =========================================================================
// GLOBAL CROSS-CORE INSTANCES (attemp at implementing spinlocks
// and removing "volatile")
// =========================================================================
// CRITICAL FIX: Removed 'volatile'. The Spinlocks will handle memory safety.
extern GlobalDataBank CurrentRobotData;
extern HardwareCommandBus HardwareCommands;
extern TeleopCommandBus TeleopCommands;

// The FreeRTOS hardware spinlocks to prevent memory tearing between cores
extern portMUX_TYPE globalDataBusLock;
extern portMUX_TYPE hardwareCmdLock;
extern portMUX_TYPE teleopCmdLock;