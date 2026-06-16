#include "tasks/Task_ControlLoop.h"
#include "core/GlobalDataBus.h" // For CurrentRobotData and spinlocks
#include "config/SystemConfig.h"
#include "utils/RemoteLogger.h"

extern RemoteLogger logger; // Logger is safely thread-safe, so extern is okay here

// ==========================================
// TASK 2: THE MAIN CONTROL LOO
// (CORE 1 - APP CPU)
// ==========================================

void ControlLoopTask(void *pvParameters) { 
    // 1. Unpack the dependencies from main.cpp
    ControlLoopContext* ctx = static_cast<ControlLoopContext*>(pvParameters);
    
    const TickType_t xFrequency = pdMS_TO_TICKS(SystemConfig::MAIN_LOOP_TICK_RATE_MS);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    static bool wasBleConnected = false;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // --- THE TELEOPERATION OVERRIDE WATCHDOG ---
        TeleopCommandBus teleopSnapshot;
        portENTER_CRITICAL(&teleopCmdLock);
        teleopSnapshot = TeleopCommands;
        portEXIT_CRITICAL(&teleopCmdLock);

        // 2. Evaluate connection state
        // (No changeMode needed here as its handled by BehaviourEngine)
        if (teleopSnapshot.isConnected && !wasBleConnected) {
            wasBleConnected = true;
            SysConfig.BRAIN_ACTIVE = false;    // Shut down autonomous decision engine
            logger.println("[SYSTEM] BLE Connected. Manual Override Engaged.");
        } 
        else if (!teleopSnapshot.isConnected && wasBleConnected) {
            wasBleConnected = false;
            SysConfig.BRAIN_ACTIVE = true;     // Turn autonomous brain back on
            logger.println("[SYSTEM] BLE Disconnected. Autonomous Brain Resumed.");
        }
        
        // --- SNAPSHOT & BRAIN UPDATE ---
        GlobalDataBank physicsSnapshot;
        portENTER_CRITICAL(&globalDataBusLock);
        physicsSnapshot = CurrentRobotData; 
        portEXIT_CRITICAL(&globalDataBusLock);

        // ==========================================
        // 3. PLAN (Run Brain & Kinematics Math)
        // ==========================================
        int16_t finalLeftPWM = 0;
        int16_t finalRightPWM = 0;

        if (physicsSnapshot.health.hardwareBitmask & Comms::HealthBit::IMU_OK) {
            // This runs the state machine. The active mode will internally 
            // call ctx->kinematics->navigateToHeading(), doing the math!
            ctx->brain->update(physicsSnapshot); 
            
            // Pull the answers from the math engine
            finalLeftPWM = ctx->kinematics->getLeftPWM();
            finalRightPWM = ctx->kinematics->getRightPWM();
        } else {
            // IMU Failure Failsafe
            ctx->kinematics->stop();
        }

        // ==========================================
        // 4. SAFELY WRITE INTENT TO CENTRAL NERVOUS SYSTEM
        // ==========================================
        portENTER_CRITICAL(&globalDataBusLock);
        CurrentRobotData.physics.leftMotorPWM = finalLeftPWM;
        CurrentRobotData.physics.rightMotorPWM = finalRightPWM;
        CurrentRobotData.controlDebug.targetHeading = ctx->kinematics->getTargetHeading();
        CurrentRobotData.controlDebug.headingError = ctx->kinematics->getHeadingError();
        portEXIT_CRITICAL(&globalDataBusLock);

        // ==========================================
        // 5. ACT (Pulse the Hardware)
        // ==========================================
        if (ctx->motorDriver) {
            // THIS is the only place hardware is touched!
            ctx->motorDriver->drive(finalLeftPWM, finalRightPWM);
        }
    }
}