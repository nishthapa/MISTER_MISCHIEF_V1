#pragma once
#include "hal/interfaces/I_IMU.h"
#include <stdint.h>
#include "config/PacketIDRegistry.h" // <-- Brings in your Comms::HealthBit definitions!

// =========================================================================
// 1. OUTBOUND TELEMETRY (The Absolute Truth of the Robot)
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

// The Master Wrapper
struct GlobalDataBank {
    PhysicsState physics;
    SensorState sensors;
    SystemHealthState health;
};

// =========================================================================
// 2. INBOUND COMMANDS (Internal Routing)
// =========================================================================
struct HardwareCommandBus {
    bool requestGyroCalibration = false;
    bool requestAccelCalibration = false;
    bool requestMagCalibration = false;
    float targetFilterBeta = 0.0f;
};

struct TeleopCommandBus {
    float joyX = 0.0f;       // -1.0f (Left) to 1.0f (Right)
    float joyY = 0.0f;       // -1.0f (Reverse) to 1.0f (Forward)
    bool usePIDDrive = false; // Toggles between RAW motor mixing and PID Heading Hold
    bool isConnected = false; // Failsafe: Stops robot if BLE drops
};

// =========================================================================
// GLOBAL CROSS-CORE INSTANCES
// =========================================================================
extern volatile GlobalDataBank CurrentRobotData;
extern volatile HardwareCommandBus HardwareCommands;
extern volatile TeleopCommandBus TeleopCommands;