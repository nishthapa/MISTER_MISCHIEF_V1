#include <Arduino.h>

#include "config/ConfigurationManager.h"
#include "config/PinConfig.h"         
#include "hal/factories/MotorDriverFactory.h"
#include "hal/factories/IMUFactory.h"
#include "hal/factories/DistanceSensorFactory.h"          
#include "objectproviders/PIDControllerFactory.h" 
#include "behaviours/Mode_NormalDriving.h"
#include "behaviours/Mode_CompassLock.h"
#include "behaviours/Mode_MaintainDistance.h"
#include "behaviours/Mode_ObstacleAvoidance.h"
#include "behaviours/Mode_Dizzy.h"
#include "behaviours/Mode_DeepSleep.h"
#include "behaviours/Mode_AutoTune.h"
#include "core/BehaviourEngine.h"
#include "core/KinematicsEngine.h"
#include "utils/RadioManager.h"
#include "utils/RemoteLogger.h"
#include "config/CommandRegistry.h" 
#include "core/CommandProcessor.h"
#include "config/SystemConfig.h"
#include "core/RobotState.h"
#include "core/GlobalSensorState.h" // <--- THE NEW INCLUDE

RemoteLogger logger(SystemConfig::WEBSOCKET_PORT); 

// ==========================================
// INTER-CORE COMMUNICATION (THE BRIDGE)
// ==========================================
volatile SystemMode GLOBAL_MODE = SystemMode::BOOTING;

// Instantiate the global memory bank!
volatile GlobalSensorState CurrentSensorState = { -1.0f, {0,0,0,0,false,0}, false };

// Instantiate the low level Hardware Command Bus for sending commands from the Brain to the HAL without direct hardware access!
volatile HardwareCommandBus HardwareCommands = { false, false, false, 0.1f }; // FIX: Added the extra falses for Accel and Mag!

// ==========================================
// GLOBAL HARDWARE OBJECTS
// ==========================================
// Notice: We still instantiate them here, but the Brain won't use them directly anymore!
I_DistanceSensor* frontDistanceSensor = DistanceSensorFactory::createDistanceSensor();
I_MotorDriver* motorDriver = MotorDriverFactory::createMotorDriver();
I_IMU* imu = IMUFactory::createIMU();

// ==========================================
// GLOBAL PID CONTROLLERS & KINEMATICS
// ==========================================
PIDController pointTurnPID = PIDControllerFactory::createPointTurnPID();
PIDController arcTurnPID   = PIDControllerFactory::createArcTurnPID();
PIDController distancePID = PIDControllerFactory::createDistanceHoldPID();

KinematicsEngine kinematics(motorDriver, &pointTurnPID, &arcTurnPID);

// ==========================================
// GLOBAL MODE OBJECTS
// ==========================================
Mode_ObstacleAvoidance obstacleMode(&kinematics); // Removed 'frontDistanceSensor' and imu
Mode_NormalDriving normalMode(&kinematics);  // Removed 'imu'
Mode_CompassLock compassMode(&kinematics); // Removed 'imu'
Mode_MaintainDistance distanceMode(&kinematics, &distancePID); // Removed 'frontDistanceSensor'
Mode_Dizzy dizzyMode(&kinematics); // standardized dependency injection from direct motor driver access level to kinematics engine level
Mode_DeepSleep sleepMode(&kinematics); // standardized dependency injection from direct motor driver access level to kinematics engine level
Mode_AutoTune autotuneMode(&kinematics); // Removed 'imu'

// ==========================================
// MODE SWITCHER (The Brain)
// ==========================================
// Note: It still takes the hardware pointers right now, but we will remove them in the next step!
BehaviourEngine brain(&obstacleMode, &normalMode, &compassMode, &distanceMode, &dizzyMode, &sleepMode);
CommandProcessor cliEngine;


// ==========================================
// TASK 1: THE GATHERER (CORE 1 - APP CPU)
// ==========================================
void SensorTask(void *pvParameters) {
    unsigned long lastSonarTime = 0;

    for (;;) {
        // ==========================================
        // 0. EXECUTE HARDWARE COMMANDS FROM THE BRAIN
        // ==========================================
        if (HardwareCommands.requestGyroCalibration) {
            if (imu) imu->calibrateGyro();
            HardwareCommands.requestGyroCalibration = false; // Reset the flag!
        }

        if (HardwareCommands.requestAccelCalibration) {
            if (imu) imu->calibrateAccel();
            HardwareCommands.requestAccelCalibration = false; // Reset the flag!
        }

        if (HardwareCommands.requestMagCalibration) {
            if (imu) imu->calibrateMag();
            HardwareCommands.requestMagCalibration = false; // Reset the flag!
        }
        
        static float currentBeta = -1.0f;
        if (currentBeta != HardwareCommands.targetFilterBeta) {
            currentBeta = HardwareCommands.targetFilterBeta;
            if (imu) imu->setFilterBeta(currentBeta);
        }

        // ==========================================
        // 1. ISOLATED HARDWARE POLLING
        // ==========================================
        
        // Poll the IMU as fast as possible (every frame)
        if (CurrentSensorState.imuAlive) {
            FusedAngles currentAngles = imu->getAngles();
            
            // C++ volatile struct fix: We must assign the fields manually!
            CurrentSensorState.imuAngles.yaw = currentAngles.yaw;
            CurrentSensorState.imuAngles.pitch = currentAngles.pitch;
            CurrentSensorState.imuAngles.roll = currentAngles.roll;
            CurrentSensorState.imuAngles.gForce = currentAngles.gForce;
            CurrentSensorState.imuAngles.hasCompass = currentAngles.hasCompass;
            CurrentSensorState.imuAngles.compassHeading = currentAngles.compassHeading;
        }

        unsigned long currentTime = millis();

        // Poll the Sonar exactly every 50ms
        if (currentTime - lastSonarTime >= 50) { 
            lastSonarTime = currentTime;
            CurrentSensorState.distanceCM = frontDistanceSensor->getDistanceCM();
        }

        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}


// ==========================================
// TASK 2: THE MAIN CONTROL LOOP (CORE 1 - APP CPU)
// ==========================================
void ControlLoopTask(void *pvParameters) { 
    const TickType_t xFrequency = pdMS_TO_TICKS(SystemConfig::MAIN_LOOP_TICK_RATE_MS);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // THE ISOLATION: The Brain only runs if the Gatherer says the IMU is alive!
        if (CurrentSensorState.imuAlive) {
            // WE NOW PASS THE MEMORY STATE TO THE BRAIN!
            brain.update(CurrentSensorState); 
        } else {
            motorDriver->stop(); 
        }
    }
}


// ==========================================
// TASK 3: TELEMETRY PRODUCER (CORE 1 - APP CPU)
// ==========================================
void TelemetryTask(void *pvParameters) {
    unsigned long lastTelemetryTime = 0;
    for (;;) {
        // Read CLI input securely
        while (Serial.available()) {
            cliEngine.processChar(Serial.read());
        }

        unsigned long currentTime = millis();
        
        // Format the JSON and drop it in the FreeRTOS Queue
        if (currentTime - lastTelemetryTime >= SystemConfig::TELEMETRY_PING_DELAY_MS) {
            lastTelemetryTime = currentTime;
            if (Config.SERIAL_DEBUG_MASTER && CurrentSensorState.imuAlive) {
                logger.sendTelemetryJSON("{\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,\"sonar\":%.1f,\"mode\":\"%s\",\"brain\":%s}\n", 
                              CurrentSensorState.imuAngles.yaw, 
                              CurrentSensorState.imuAngles.pitch,
                              CurrentSensorState.imuAngles.roll,
                              CurrentSensorState.distanceCM,
                              brain.getActiveModeName(), 
                              Config.BRAIN_ACTIVE ? "true" : "false");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==========================================
// TASK 4: LOW-LEVEL NETWORK CONSUMER (CORE 0 - PRO CPU)
// ==========================================
void NetworkTask(void *pvParameters) {
    for (;;) {
        // Handle incoming WebSocket handshakes natively on Core 0
        logger.handleClient();
        
        // Read the Mailbox and transmit the waiting strings over the air
        logger.processQueue();
        
        // Give the Wi-Fi driver plenty of breathing room
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}


// ==========================================
// SETUP
// ==========================================
TaskHandle_t SensorTaskHandle;
TaskHandle_t ControlLoopTaskHandle;
TaskHandle_t TelemetryTaskHandle; // FIX: Added the 3rd task handle!
TaskHandle_t NetworkTaskHandle; // FIX: Added the 4th task handle!

void setup() {
  ConfigSys.init(); 
  logger.beginSerial();
  RadioManager::initRadios();
  logger.bindRadios();

  // === THE WIFI BUFFER FIX ===
  // Prevents the modem from sleeping, keeping RX buffers flush and ready for abrupt disconnects!
  //WiFi.setSleep(false);

  pointTurnPID.setTunings(Config.PID_POINT_P, Config.PID_POINT_I, Config.PID_POINT_D, Config.PID_POINT_ILIM, Config.PID_POINT_LIM);
  arcTurnPID.setTunings(Config.PID_ARC_P, Config.PID_ARC_I, Config.PID_ARC_D, Config.PID_ARC_ILIM, Config.PID_ARC_LIM);
  distancePID.setTunings(Config.PID_DIST_P, Config.PID_DIST_I, Config.PID_DIST_D, Config.PID_DIST_ILIM, Config.PID_DIST_LIM);
  
  imu->setFilterBeta(Config.MADGWICK_FILTER_BETA);
  logger.println("Configuration Manager loaded from permanent memory.");

  unsigned long waitStart = millis();
  while (millis() - waitStart < SystemConfig::TELNET_WAIT_TIME_MS) {
      logger.handleClient(); 
      delay(50);
  }
  
  delay(SystemConfig::HARDWARE_WAKE_DELAY_MS);

  frontDistanceSensor->init();
  motorDriver->init();

  logger.println("Waking up the IMU...");
  int imuRetries = 0;
  while (!imu->init() && imuRetries < SystemConfig::IMU_MAX_RETRIES) {
      logger.println("IMU failed to initialize. Rebooting I2C bus...");
      delay(SystemConfig::IMU_RETRY_DELAY_MS); 
      imuRetries++;
  }
  
  // Set the global state!
  CurrentSensorState.imuAlive = (imuRetries < SystemConfig::IMU_MAX_RETRIES);
  
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  bool isColdBoot = (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED);
  
  brain.init(isColdBoot);

  // Temporary testing override
  Config.BRAIN_ACTIVE = false;       
  brain.changeMode(&autotuneMode);   
  
  logger.println("Mister Mischief V1 Booting...");
  delay(1000); 

  // === THE NEW 4-TASK ISOLATED ARCHITECTURE ===

  // App CPU (Core 1)
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", SystemConfig::TASK_STACK_SENSOR, NULL, SystemConfig::SENSOR_TASK_PRIORITY, &SensorTaskHandle, SystemConfig::SENSOR_TASK_CORE_AFFINITY);
  xTaskCreatePinnedToCore(ControlLoopTask, "ControlLoopTask", SystemConfig::TASK_STACK_PHYSICS, NULL, SystemConfig::CONTROL_LOOP_TASK_PRIORITY, &ControlLoopTaskHandle, SystemConfig::CONTROL_LOOP_TASK_CORE_AFFINITY);
  xTaskCreatePinnedToCore(TelemetryTask, "TelemetryTask", SystemConfig::TASK_STACK_TELEMETRY, NULL, SystemConfig::TELEMETRY_TASK_PRIORITY, &TelemetryTaskHandle, SystemConfig::TELEMETRY_TASK_CORE_AFFINITY); 
  
  // Pro CPU (Core 0) - Safely isolated!
  xTaskCreatePinnedToCore(NetworkTask, "NetworkTask", SystemConfig::TASK_STACK_NETWORK, NULL, 1, &NetworkTaskHandle, SystemConfig::NETWORK_TASK_CORE_AFFINITY);
}

void loop() { vTaskDelete(NULL); }