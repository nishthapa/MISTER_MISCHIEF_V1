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
}__attribute__((packed));

struct ActuatorState {              // MsgId 104
    int16_t leftMotorPWM = 0;       // Actuators (Left Track Current Power)
    int16_t rightMotorPWM = 0;      // Actuators (Right Track Current Power) 
    bool isDriving = false;
}__attribute__((packed));

struct SensorState {                // MsgId 110
    float distanceCM = -1.0f;       // Sonar Distance
    uint16_t batteryVoltageMV = 0;  // Future INA226 Voltage Expansion
    int16_t currentDrawMA = 0;      // Future INA226 Current Expansion
}__attribute__((packed));

struct SystemHealthState {          // MsgId 130
    uint32_t loopTimeUs = 0;        // Main physics loop speed
    uint32_t freeHeap = 0;          // RAM available
    uint16_t hardwareBitmask = 0;   // Alive components (Uses Comms::HealthBit)
    uint8_t cpu0Load = 0;           // Sensor Core Utilization %
    uint8_t cpu1Load = 0;           // Physics Core Utilization %
}__attribute__((packed));

// CONTROL DEBUG (PID & Target tracking)
struct ControlDebugState {          // MsgId 121
    float targetHeading = 0.0f; 
    float headingError = 0.0f;
    bool pidEnabled = false;
    float joyX = 0.0f; // <-- Add this!
    float joyY = 0.0f; // <-- Add this!
}__attribute__((packed));

struct EventState {                 // MsgId 135: Semantic Events (The final triggers)
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
    //bool isDriving = false;
    bool isHandTeasing = false;
    bool isHandVanishing = false;
    bool isHandling = false;
    bool hasExperiencedLift = false;
    bool isLowering = false;
    bool hasLanded = false;
    bool isDizzy = false;
}__attribute__((packed));

struct PerceptionMetrics {          // MsgId 136: Intermediate Math & Tuning Metrics
    float distanceDelta = 0.0f;
    float totalRawEnergy = 0.0f;
    float rawYawEnergy = 0.0f;
    float rawPitchEnergy = 0.0f;
    float rawRollEnergy = 0.0f;
    float currentGForce = 0.0f;
}__attribute__((packed));

// COGNITIVE STATE (MODE & MOOD)
struct CognitiveState {             // MsgId 131
    SystemMode systemMode = SystemMode::BOOTING;
    MoodState robotMood = MoodState::HAPPY; // Default mood // <--- MUST be MoodState, not RobotMood!
}__attribute__((packed));

struct SystemLogMessage {           // MsgId 140: serial.println() telemetry mirro
    char text[128] = {0}; // Fixed size in RAM, no memory leaks!
}__attribute__((packed));

struct NetworkLinkState {           // MsgId 132: WIFI/BT RSSI
    int8_t wifiRSSI = -127;         // -127 dBm is the standard "Dead Signal" value
    int8_t bleRSSI = -127; 
}__attribute__((packed));

// The Master Wrapper
struct GlobalDataBank {
    PhysicsState physics;
    ActuatorState actuators; // Split!
    SensorState sensors;
    SystemHealthState health;
    EventState events; // <-- New centralized semantic events
    PerceptionMetrics perception; // <-- The new tuning mirror in the telemetry
    CognitiveState cognition; // <-- Mode and Mood
    ControlDebugState controlDebug; // <-- For PID Target and error
    SystemLogMessage systemLog;
    NetworkLinkState networkLink; // <--- NEW: Live Signal Bars FOR WIFI/BT!
    bool hasNewLog = false; // pulse true-->false when a serial.println() message is mirrored
    bool otaUpdateStarted = false; // NEW: OTA Graceful Shutdown Flag
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

    // --- NEW: The Live-Tuning Pipeline ---
    bool reloadPIDTunings = false; 
    float incomingP = 0.0f;
    float incomingI = 0.0f;
    float incomingD = 0.0f;
    uint8_t targetPIDController = 0; // 0 = None, 1 = Point Turn, 2 = Arc Turn, 3 = Distance
};

// struct TeleopCommandBus {
//     float joyX = 0.0f;       // -1.0f (Left) to 1.0f (Right)
//     float joyY = 0.0f;       // -1.0f (Reverse) to 1.0f (Forward)
//     bool usePIDDrive = false; // Toggles between RAW motor mixing and PID Heading Hold
//     bool isConnected = false; // Failsafe: Stops robot if BLE drops
//     bool emergencyKillSwitch = false; // <--- NEW: Software instant-stop
// };


// To keep track of who is controlling
enum class TeleopMedium : uint8_t {
    NONE = 0,
    BLE = 1,
    WIFI_WS = 2,
    INTERNET = 3
};

struct TeleopCommandBus {
    float joyX = 0.0f;
    float joyY = 0.0f;
    bool usePIDDrive = false;
    
    // --- THE TOKEN AUTHORITY ---
    bool isOverrideActive = false;
    TeleopMedium activeMedium = TeleopMedium::NONE;
    unsigned long lastHeartbeat = 0;
    static constexpr unsigned long TIMEOUT_MS = 500; // 500ms Deadman Switch

    // 1. Claim the Token
    bool requestControl(TeleopMedium medium) {
        if (activeMedium == TeleopMedium::NONE || activeMedium == medium) {
            activeMedium = medium;
            isOverrideActive = true;
            lastHeartbeat = millis();
            return true;
        }
        return false; // Denied: Another medium is already driving!
    }

    // 2. Voluntarily Release the Token
    void releaseControl(TeleopMedium medium) {
        if (activeMedium == medium) {
            activeMedium = TeleopMedium::NONE;
            isOverrideActive = false;
            joyX = 0.0f; joyY = 0.0f;
            usePIDDrive = false;
        }
    }

    // 3. Receive Driving Vectors (Requires Authorization)
    bool updateCommand(TeleopMedium medium, float x, float y, bool pid) {
        if (activeMedium == medium && isOverrideActive) {
            joyX = x; joyY = y; usePIDDrive = pid;
            lastHeartbeat = millis(); // Feed the watchdog!
            return true;
        }
        return false; // Ignored: Unauthorized client tried to drive
    }

    // 4. The Deadman Switch (Watchdog)
    void checkFailsafe() {
        if (isOverrideActive && (millis() - lastHeartbeat > TIMEOUT_MS)) {
            // Client timed out! Revoke control and zero the motors.
            activeMedium = TeleopMedium::NONE;
            isOverrideActive = false;
            joyX = 0.0f; joyY = 0.0f;
            usePIDDrive = false;
        }
    }
};

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