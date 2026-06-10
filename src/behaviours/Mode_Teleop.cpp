#include "behaviours/Mode_Teleop.h"
#include "config/ConfigurationManager.h"
#include "config/SystemConfig.h" // Added to access MAIN_LOOP_TICK_RATE_MS for dt
#include <Arduino.h>

Mode_Teleop::Mode_Teleop(KinematicsEngine* k) {
    kinematics = k;
    targetHeading = 0.0f;
}

void Mode_Teleop::onEnter(const volatile GlobalDataBank& robotData) {
    kinematics->stop();
    targetHeading = robotData.physics.imuAngles.yaw; // Snapshot heading for seamless PID engagement
}

void Mode_Teleop::update(const RobotMood& currentMood, const volatile GlobalDataBank& robotData) {
    // 1. HARDWARE FAILSAFE: Immediate drop-dead if BLE link breaks
    if (!TeleopCommands.isConnected) {
        kinematics->stop();
        return;
    }

    // 2. Read raw joystick values directly from the cross-core memory bank
    float rawY = TeleopCommands.joyY; // Forward / Reverse (-1.0f to 1.0f)
    float rawX = TeleopCommands.joyX; // Left / Right (-1.0f to 1.0f)

    // 3. Hardware Deadband Filter (Cleans up mechanical centering slop)
    if (abs(rawY) < 0.10f) rawY = 0.0f;
    if (abs(rawX) < 0.10f) rawX = 0.0f;


    // --- UNIVERSAL IDLE STOP ---
    // If the joystick is centered, force an immediate hard stop.
    // This kills the motors for BOTH PID and Raw Arcade modes.
    // Instant kill switch. Also flush the filter memory so we don't carry old data!
    if (rawX == 0.0f && rawY == 0.0f) {
        smoothedX = 0.0f;
        smoothedY = 0.0f;
        kinematics->stop(); 
        // Reset memory so the robot doesn't aggressively snap back to an old target when you touch the stick again!
        targetHeading = robotData.physics.imuAngles.yaw; 
        return; 
    }

    // --- THE FIX: EXPONENTIAL MOVING AVERAGE (EMA) FILTER ---
    // Alpha controls responsiveness. 0.75 means we trust the new raw input 75%, 
    // and trust the old smoothed history 25%. 
    // increase for stick response, decrease for smoothness.
    float filterAlpha = 0.75f; 
    smoothedX = (filterAlpha * rawX) + ((1.0f - filterAlpha) * smoothedX);
    smoothedY = (filterAlpha * rawY) + ((1.0f - filterAlpha) * smoothedY);

    // Re-assign to standard variables for the rest of the logic
    float x = smoothedX;
    float y = smoothedY;

    // 4. Operational Routing Logic
    if (TeleopCommands.usePIDDrive) {
        // --- PID-ASSISTED DRIVE (Heading Hold) ---
        float dt = SystemConfig::MAIN_LOOP_TICK_RATE_MS / 1000.0f;

        // 1. THE AGGRESSIVE REQUEST
        // We ask for a massively fast turn (e.g., 400 deg/sec).
        // This guarantees the target instantly hits the end of the leash, 
        // making the joystick feel razor-sharp and perfectly responsive.
        float requestedTurnRate = 400.0f; 
        targetHeading += (x * requestedTurnRate * dt);
        
        if (targetHeading > 180.0f)  targetHeading -= 360.0f;
        if (targetHeading < -180.0f) targetHeading += 360.0f;

        // 2. THE DYNAMIC IMU THROTTLE (The Leash)
        float error = targetHeading - robotData.physics.imuAngles.yaw;
        
        if (error > 180.0f) error -= 360.0f;
        if (error < -180.0f) error += 360.0f;

        // 2. THE DYNAMIC IMU THROTTLE (The Fix!)
        // Scale the maximum allowed error based on how hard the joystick is pushed.
        // Full stick = 45.0 degrees of error (Full Power)
        // Small stick (0.13) = 5.85 degrees of error (Gentle Power)
        float maxLeashDegrees = abs(x) * 45.0f;
        if (error > maxLeashDegrees) {
            targetHeading = robotData.physics.imuAngles.yaw + maxLeashDegrees;
        } else if (error < -maxLeashDegrees) {
            targetHeading = robotData.physics.imuAngles.yaw - maxLeashDegrees;
        }
        
        if (targetHeading > 180.0f)  targetHeading -= 360.0f;
        if (targetHeading < -180.0f) targetHeading += 360.0f;

        float baseSpeedUncapped = y * 100.0f; 

        kinematics->navigateToHeading(targetHeading, robotData.physics.imuAngles.yaw, baseSpeedUncapped, 1.0f);
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