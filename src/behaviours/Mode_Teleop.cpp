#include "behaviours/Mode_Teleop.h"
#include "config/ConfigurationManager.h"
#include <Arduino.h>

Mode_Teleop::Mode_Teleop(KinematicsEngine* k) {
    kinematics = k;
    targetHeading = 0.0f;
}

void Mode_Teleop::onEnter(const volatile GlobalSensorState& sensorState) {
    kinematics->stop();
    targetHeading = sensorState.imuAngles.yaw; // Snapshot heading for seamless PID engagement
}

void Mode_Teleop::update(const RobotMood& currentMood, const volatile GlobalSensorState& sensorState) {
    // 1. HARDWARE FAILSAFE: Immediate drop-dead if BLE link breaks
    if (!TeleopCommands.isConnected) {
        kinematics->stop();
        return;
    }

    // 2. Read raw joystick values directly from the cross-core memory bank
    float y = TeleopCommands.joyY; // Forward / Reverse (-1.0f to 1.0f)
    float x = TeleopCommands.joyX; // Left / Right (-1.0f to 1.0f)

    // 3. Hardware Deadband Filter (Cleans up mechanical centering slop)
    if (abs(y) < 0.10f) y = 0.0f;
    if (abs(x) < 0.10f) x = 0.0f;

    // --- THE FIX: UNIVERSAL IDLE STOP ---
    // If the joystick is centered, force an immediate hard stop.
    // This kills the motors for BOTH PID and Raw Arcade modes.
    if (x == 0.0f && y == 0.0f) {
        kinematics->stop(); 
        return; // Exit here. The code below won't run.
    }

    // 4. Operational Routing Logic
    if (TeleopCommands.usePIDDrive) {
        // --- PID-ASSISTED DRIVE (Heading Hold) ---
        // Modify the target yaw dynamically using the X axis (5 degrees per loop tick)
        targetHeading += (x * 5.0f);
        
        // Keep heading bound perfectly within rolling plane wrap-around (-180 to 180)
        if (targetHeading > 180.0f)  targetHeading -= 360.0f;
        if (targetHeading < -180.0f) targetHeading += 360.0f;

        // Uncapped baseline forward velocity mapped directly to full scale
        float baseSpeedUncapped = y * 100.0f; 

        // We explicitly pass 1.0f for moodAggression to bypass the personality core completely
        kinematics->navigateToHeading(targetHeading, sensorState.imuAngles.yaw, baseSpeedUncapped, 1.0f);
    } 
    else {
        // --- RAW OPEN-LOOP MIXING (Arcade Drive) ---
        // Standard kinematic differential mixing for tank steering
        float left = y + x;
        float right = y - x;

        // Normalize values back to 1.0f maximum if vector additions overload full scale
        float max_val = max(abs(left), abs(right));
        if (max_val > 1.0f) {
            left /= max_val;
            right /= max_val;
        }

        // Apply raw hardware limits (100% duty cycle availability) 
        // Completely stripped of speed multipliers or software dampeners
        float rawLeftMax = left * 100.0f;
        float rawRightMax = right * 100.0f;

        kinematics->rawDrive(rawLeftMax, rawRightMax);
    }
}

void Mode_Teleop::onExit() {
    kinematics->stop(); // Ensure motors kill power if mode hot-swaps
}