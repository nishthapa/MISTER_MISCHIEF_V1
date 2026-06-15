#include "behaviours/Mode_Teleop.h"
#include "config/ConfigurationManager.h"
#include "config/SystemConfig.h" // Added to access MAIN_LOOP_TICK_RATE_MS for dt
#include "core/GlobalDataBus.h"
#include <Arduino.h>

Mode_Teleop::Mode_Teleop(KinematicsEngine* k) {
    kinematics = k;
    targetHeading = 0.0f;
}

void Mode_Teleop::onEnter(const GlobalDataBank& robotData) { // Removed volatile
    kinematics->stop();
    targetHeading = robotData.physics.imuAngles.yaw;
    // Snapshot heading for seamless PID engagement
}

void Mode_Teleop::update(const RobotMood& currentMood, const GlobalDataBank& robotData) {
    
    // --- THREAD SAFETY: Take a clean snapshot of the incoming commands ---
    TeleopCommandBus teleopSnapshot;
    portENTER_CRITICAL(&teleopCmdLock);
    teleopSnapshot = TeleopCommands;
    portEXIT_CRITICAL(&teleopCmdLock);
    // ---------------------------------------------------------------------

    // 1. HARDWARE FAILSAFE: Immediate drop-dead if BLE link breaks
    if (!teleopSnapshot.isConnected) {
        kinematics->stop();
        return;
    }

    // 2. Read raw joystick values directly from the cross-core memory bank
    //float rawY = TeleopCommands.joyY;
    // Forward / Reverse (-1.0f to 1.0f)
    // float rawX = TeleopCommands.joyX;
    // Left / Right (-1.0f to 1.0f)

    // 2. Read raw joystick values directly from the safe local snapshot
    float rawY = teleopSnapshot.joyY;
    // Forward / Reverse (-1.0f to 1.0f)
    float rawX = teleopSnapshot.joyX;
    // Left / Right (-1.0f to 1.0f)



    // 3. Hardware Deadband Filter (Cleans up mechanical centering slop)
    if (abs(rawY) < 0.10f) rawY = 0.0f;
    if (abs(rawX) < 0.10f) rawX = 0.0f;

    // --- UNIVERSAL IDLE STOP ---
    // If the joystick is centered, force an immediate hard stop.
    // This kills the motors for BOTH PID and Raw Arcade modes.
    // Instant kill switch.
    // Also flush the filter memory so we don't carry old data!
    if (rawX == 0.0f && rawY == 0.0f) {
        smoothedX = 0.0f;
        smoothedY = 0.0f;
        
        // FIX 1: Only completely kill the robot if we are in manual Arcade mode.
        // If PID drive is ON, we want to bypass this return so the heading-hold PID 
        // can actively run and fight being pushed around while parked!
        if (!teleopSnapshot.usePIDDrive) {
            kinematics->stop(); 
            // Reset memory so the robot doesn't aggressively snap back to an old target when you touch the stick again!
            targetHeading = robotData.physics.imuAngles.yaw; 
            return; 
        }
    }

    // --- THE FIX: EXPONENTIAL MOVING AVERAGE (EMA) FILTER ---
    // Alpha controls responsiveness.
    // 0.75 means we trust the new raw input 75%, 
    // and trust the old smoothed history 25%.
    // increase for stick response, decrease for smoothness.
    float filterAlpha = 0.75f;
    smoothedX = (filterAlpha * rawX) + ((1.0f - filterAlpha) * smoothedX);
    smoothedY = (filterAlpha * rawY) + ((1.0f - filterAlpha) * smoothedY);
    
    // Re-assign to standard variables for the rest of the logic
    float x = smoothedX;
    float y = smoothedY;

    // 4. Operational Routing Logic
    if (teleopSnapshot.usePIDDrive) {
        // --- PID-ASSISTED DRIVE (Heading Hold) ---
        float dt = SystemConfig::MAIN_LOOP_TICK_RATE_MS / 1000.0f;
        
        // 1. RC EXPO (Blended Polynomial Curve)
        // 1.0 = Pure Cubic (Very dead center)
        // 0.0 = Pure Linear (Extremely twitchy)
        // Try starting at 0.60 or 0.65 for tracked robots!
        // Increase to make stick center more lenient
        float steeringExpoWeight = 0.60f;  // STEERING EXPO (X-Axis) // Todo: Migrate expo to NVS SysConfig
        
        // The Magic Formula: Blend the cubic curve with the linear straight line
        float xExpo = (steeringExpoWeight * (x * x * x)) + ((1.0f - steeringExpoWeight) * x);

        // Restore the massive 400 deg/sec request! 
        float requestedTurnRate = 350.0f; 
        targetHeading += (xExpo * requestedTurnRate * dt);
        
        if (targetHeading > 180.0f)  targetHeading -= 360.0f;
        if (targetHeading < -180.0f) targetHeading += 360.0f;

        // 2. THE DYNAMIC IMU THROTTLE (The Leash)
        float error = targetHeading - robotData.physics.imuAngles.yaw;
        if (error > 180.0f) error -= 360.0f;
        if (error < -180.0f) error += 360.0f;
        
        // 2. THE STATIC ANTI-WINDUP LEASH (The Fix!)
        // A static 60-degree leash prevents the 180-degree wrap-around stutter, 
        // but is wide enough that a physical bump (e.g., 20-30 degrees) will NOT 
        // trigger the clamp. The robot retains its memory of the target path!
        float maxLeashDegrees = 60.0f;
        
        if (error > maxLeashDegrees) {
            targetHeading = robotData.physics.imuAngles.yaw + maxLeashDegrees;
        } else if (error < -maxLeashDegrees) {
            targetHeading = robotData.physics.imuAngles.yaw - maxLeashDegrees;
        }
        
        if (targetHeading > 180.0f)  targetHeading -= 360.0f;
        if (targetHeading < -180.0f) targetHeading += 360.0f;

        // 1. RC EXPO (Blended Polynomial Curve)
        // 1.0 = Pure Cubic (Very dead center)
        // 0.0 = Pure Linear (Extremely twitchy)
        // Try starting at 0.60 or 0.65 for tracked robots!
        // Increase to make stick center more lenient
        float throttleExpoWeight = 0.30f; // THROTTLE EXPO (X-Axis) // Todo: Migrate expo to NVS SysConfig

        float yExpo = (throttleExpoWeight * (y * y * y)) + ((1.0f - throttleExpoWeight) * y);

        // Apply the newly smoothed Y-curve to the motors
        float baseSpeedUncapped = yExpo * 100.0f;

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
    kinematics->stop();
    // Ensure motors kill power if mode hot-swaps
}