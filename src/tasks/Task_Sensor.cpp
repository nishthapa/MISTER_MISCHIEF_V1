#include "tasks/Task_Sensor.h"
#include "core/GlobalDataBus.h"
#include "config/SystemConfig.h"
#include "config/SensorConfig.h"
#include "utils/RemoteLogger.h"

// ==========================================
// TASK 1: THE GATHERER (CORE 1 - APP CPU)
// ==========================================

void SensorTask(void *pvParameters) {
    SensorContext* ctx = static_cast<SensorContext*>(pvParameters);

    const TickType_t xFrequency = pdMS_TO_TICKS(SystemConfig::MAIN_LOOP_TICK_RATE_MS);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    unsigned long lastSonarTime = 0;

    // --- NEW: SONAR Health Monitoring State ---
    uint8_t consecutiveSonarFailures = 0;
    const uint8_t MAX_SONAR_FAILURES = 20; // Allow 20 dropped pings before declaring death

    for (;;) {
        // Start the CPU stopwatch at the very beginning of the loop to start counting Loop Time!
        uint32_t loopStartUs = micros();
        // ==========================================
        // 🚨 OTA GRACEFUL SHUTDOWN CHECK 🚨
        // ==========================================
        bool isOTAActive = false;
        portENTER_CRITICAL(&globalDataBusLock);
        isOTAActive = CurrentRobotData.otaUpdateStarted;
        portEXIT_CRITICAL(&globalDataBusLock);

        if (isOTAActive) {
            logger.println("[SENSOR] OTA Detected. I2C Bus Released. Task Suspended.");
            portENTER_CRITICAL(&globalDataBusLock);
            CurrentRobotData.health.hardwareBitmask &= ~Comms::HealthBit::I2C_BUS_OK;
            portEXIT_CRITICAL(&globalDataBusLock);
            vTaskSuspend(NULL); 
        }

        // ==========================================
        // 0. SAFELY EXECUTE HARDWARE COMMANDS
        // ==========================================
        HardwareCommandBus cmdSnapshot;
        portENTER_CRITICAL(&hardwareCmdLock);
        cmdSnapshot = HardwareCommands;
        portEXIT_CRITICAL(&hardwareCmdLock);

        if (cmdSnapshot.requestGyroCalibration) {
            if (ctx->imu) ctx->imu->calibrateGyro();
            portENTER_CRITICAL(&hardwareCmdLock);
            HardwareCommands.requestGyroCalibration = false; 
            portEXIT_CRITICAL(&hardwareCmdLock);
        }

        if (cmdSnapshot.requestAccelCalibration) {
            if (ctx->imu) ctx->imu->calibrateAccel();
            portENTER_CRITICAL(&hardwareCmdLock);
            HardwareCommands.requestAccelCalibration = false; 
            portEXIT_CRITICAL(&hardwareCmdLock);
        }

        if (cmdSnapshot.requestMagCalibration) {
            if (ctx->imu) ctx->imu->calibrateMag();
            portENTER_CRITICAL(&hardwareCmdLock);
            HardwareCommands.requestMagCalibration = false; 
            portEXIT_CRITICAL(&hardwareCmdLock);
        }
        
        static float currentBeta = -1.0f;
        if (currentBeta != cmdSnapshot.targetFilterBeta) {
            currentBeta = cmdSnapshot.targetFilterBeta;
            if (ctx->imu) ctx->imu->setFilterBeta(currentBeta);
        }

        // ==========================================
        // 1. ISOLATED HARDWARE POLLING
        // ==========================================
        if (ctx->imu) {
            FusedAngles currentAngles = ctx->imu->getAngles();
            portENTER_CRITICAL(&globalDataBusLock);
            CurrentRobotData.physics.imuAngles.yaw = currentAngles.yaw;
            CurrentRobotData.physics.imuAngles.pitch = currentAngles.pitch;
            CurrentRobotData.physics.imuAngles.roll = currentAngles.roll;
            CurrentRobotData.physics.imuAngles.gForce = currentAngles.gForce;
            CurrentRobotData.physics.imuAngles.hasCompass = currentAngles.hasCompass;
            CurrentRobotData.physics.imuAngles.compassHeading = currentAngles.compassHeading;
            portEXIT_CRITICAL(&globalDataBusLock);
        }

        unsigned long currentTime = millis();
        // Poll the Sonar exactly every 50ms
        if (currentTime - lastSonarTime >= 50) { 
            lastSonarTime = currentTime;
            if (ctx->sonar) {
                // 1. Wait for the physical sound wave OUTSIDE the spinlock
                float currentDistance = ctx->sonar->getDistanceCM(); 

                // 2. Lock the bus only for the microsecond it takes to save the variable
                portENTER_CRITICAL(&globalDataBusLock); 
                CurrentRobotData.sensors.distanceCM = currentDistance;
                portEXIT_CRITICAL(&globalDataBusLock);

                // --- NEW: Continuous Sonar Health Check ---
                //if (currentDistance < 0.0f) {
                if (currentDistance >= DistanceSensorConfig::SONAR_MAX_DIST) {
                    // Ping timed out (unplugged or blocked)
                    if (consecutiveSonarFailures < MAX_SONAR_FAILURES) {
                        consecutiveSonarFailures++;
                    }
                    // If it's been dead for 5 consecutive loops, drop the OK bit
                    if (consecutiveSonarFailures >= MAX_SONAR_FAILURES) {
                        portENTER_CRITICAL(&globalDataBusLock);
                        CurrentRobotData.health.hardwareBitmask &= ~Comms::HealthBit::SONAR_OK;
                        portEXIT_CRITICAL(&globalDataBusLock);
                    }
                } else {
                    // Ping succeeded! Reset the failure counter and assert the OK bit
                    consecutiveSonarFailures = 0;
                    portENTER_CRITICAL(&globalDataBusLock);
                    CurrentRobotData.health.hardwareBitmask |= Comms::HealthBit::SONAR_OK;
                    portEXIT_CRITICAL(&globalDataBusLock);
                }    
            }
        }

        // 🚨 STOP CPU 0 STOPWATCH
        uint32_t loopEndUs = micros();
        uint32_t activeTimeUs = loopEndUs - loopStartUs;

        // Write the CPU load to the Global Bus
        portENTER_CRITICAL(&globalDataBusLock);
        CurrentRobotData.health.cpu0Load = (activeTimeUs * 100) / (SystemConfig::MAIN_LOOP_TICK_RATE_MS * 1000);
        portEXIT_CRITICAL(&globalDataBusLock);

        vTaskDelay(pdMS_TO_TICKS(1));
        //vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}