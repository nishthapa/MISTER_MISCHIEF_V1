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

        if (physicsSnapshot.health.hardwareBitmask & Comms::HealthBit::IMU_OK) {
            ctx->brain->update(physicsSnapshot); // <-- Using ctx pointer
        } else {
            ctx->motorDriver->stop();            // <-- Using ctx pointer
        }
    }
}