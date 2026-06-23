#include "tasks/Task_Sensor.h"
#include "core/GlobalDataBus.h"
#include "config/SystemConfig.h"
#include "utils/RemoteLogger.h"

// ==========================================
// TASK 1: THE GATHERER (CORE 1 - APP CPU)
// ==========================================

void SensorTask(void *pvParameters) {
    SensorContext* ctx = static_cast<SensorContext*>(pvParameters);

    const TickType_t xFrequency = pdMS_TO_TICKS(SystemConfig::MAIN_LOOP_TICK_RATE_MS);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    unsigned long lastSonarTime = 0;

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