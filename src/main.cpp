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
#include "core/BehaviourEngine.h"
#include "config/WiFiConfig.h"
#include "utils/RadioManager.h"
#include "utils/RemoteLogger.h"
#include "config/DebugConfig.h" 
#include "config/CommandRegistry.h" 
#include "core/CommandProcessor.h"

#include "config/SystemConfig.h"

RemoteLogger logger(NetworkConfig::TELNET_PORT); 

// ==========================================
// INTER-CORE COMMUNICATION (THE BRIDGE)
// ==========================================
volatile float global_frontDistanceCM = -1.0;
volatile float global_yaw = 0.0;
volatile float global_pitch = 0.0;
volatile float global_roll = 0.0;
volatile bool global_imuAlive = false; 

// ==========================================
// GLOBAL HARDWARE OBJECTS
// ==========================================
I_DistanceSensor* frontDistanceSensor = DistanceSensorFactory::createDistanceSensor();
I_MotorDriver* motorDriver = MotorDriverFactory::createMotorDriver();
I_IMU* imu = IMUFactory::createIMU();

// ==========================================
// GLOBAL PID CONTROLLERS
// ==========================================
PIDController obstacleAvoidancePID = PIDControllerFactory::createObstacleAvoidanceNewPathScanSweepPID();
PIDController headingPID = PIDControllerFactory::createHeadingHoldPID();
PIDController compassPID = PIDControllerFactory::createCompassLockPID();
PIDController distancePID = PIDControllerFactory::createDistanceHoldPID();

// ==========================================
// GLOBAL MODE OBJECTS
// ==========================================
Mode_ObstacleAvoidance obstacleMode(motorDriver, frontDistanceSensor, imu, &obstacleAvoidancePID);

// THE UPGRADED CONSTRUCTOR IS NOW READY!
Mode_NormalDriving normalMode(motorDriver, imu, &headingPID); 

Mode_CompassLock compassMode(imu, motorDriver, &compassPID);
Mode_MaintainDistance distanceMode(frontDistanceSensor, motorDriver, &distancePID);
Mode_Dizzy dizzyMode(motorDriver);
Mode_DeepSleep sleepMode(motorDriver);


// ==========================================
// MODE / MOOD SWITCHER
// ==========================================
BehaviourEngine brain(imu, frontDistanceSensor, &obstacleMode, &normalMode, &compassMode, &distanceMode, &dizzyMode, &sleepMode);

// ==========================================
// THE COMMAND LINE ENGINE
// ==========================================
CommandProcessor cliEngine;

// ==========================================
// CORE 1: THE MAIN CONTROL LOOP
// ==========================================
void ControlLoopTask(void *pvParameters) { 
    const TickType_t xFrequency = pdMS_TO_TICKS(SystemConfig::MAIN_LOOP_TICK_RATE_MS);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // THE NEW KILLSWITCH LOGIC IF MASTER DEBUG IS OFF WHEN WE ARE TUNING/CHANGING PARAMETERS/SETTINGS
        if (!Config.BRAIN_ACTIVE) {
            motorDriver->stop(); // Force motors off
            continue;            // Skip the rest of the loop entirely!
        }

        if (global_imuAlive) {
            brain.update();
        } else {
            motorDriver->stop();
        }

        // Update Telemetry Bridge
        FusedAngles currentAngles = imu->getAngles();
        global_yaw = currentAngles.yaw;
        global_pitch = currentAngles.pitch;
        global_roll = currentAngles.roll;
    }
}

// ==========================================
// CORE 0: SENSOR READS & HIGH-SPEED CLI
// ==========================================
void SensorTask(void *pvParameters) {
  // Two separate stopwatches!
  unsigned long lastTelemetryTime = 0;
  unsigned long lastSonarTime = 0;

  for (;;) {
    logger.handleClient();
    
    // ==========================================
    // 1. INSTANT CLI PROCESSING (Runs every 1ms)
    // ==========================================
    while (Serial.available()) {
        // Feed raw bytes to the isolated Terminal Emulator!
        cliEngine.processChar(Serial.read()); 
    }

    unsigned long currentTime = millis();

    // ==========================================
    // 2. SONAR PHYSICS (Strictly limited to 50ms = 20Hz)
    // ==========================================
    // Allowing 50ms ensures all sound waves from the previous ping have safely died out!
    if (currentTime - lastSonarTime >= 50) { 
        lastSonarTime = currentTime;
        global_frontDistanceCM = frontDistanceSensor->getDistanceCM();
    }

    // ==========================================
    // 3. TELEMETRY PRINTING 
    // ==========================================
    if (currentTime - lastTelemetryTime >= SystemConfig::TELEMETRY_PING_DELAY_MS) {
        lastTelemetryTime = currentTime;
        
        if (Config.SERIAL_DEBUG_MASTER) {
            if (Config.SERIAL_DEBUG_SONAR) {
                logger.printf("[SONAR] Distance: %.1f cm | ", global_frontDistanceCM);
            }
            
            if (global_imuAlive) {
                logger.printf("Y: %5.1f | MODE: %s | BRAIN: %s (Press ENTER to pause)\n", 
                              global_yaw, 
                              brain.getActiveModeName(), 
                              Config.BRAIN_ACTIVE ? "ON" : "OFF");
            }
        } else {
            if (!global_imuAlive && Config.BRAIN_ACTIVE) motorDriver->stop(); 
        }
    }
    
    // The task rests for just 1ms so typing is instant!
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}

// ==========================================
// SETUP: INITIALIZE HARDWARE & SCHEDULER
// ==========================================
TaskHandle_t SensorTaskHandle;
TaskHandle_t ControlLoopTaskHandle;

void setup() {
  // === BOOT THE ESP32 NVS HARD DRIVE ===
  ConfigSys.init(); 
  logger.println("Configuration Manager loaded from permanent memory.");

  logger.beginSerial();
  RadioManager::initRadios();
  logger.bindRadios();

// ==========================================
  // THE TELNET WAITING ROOM
  // ==========================================
  unsigned long waitStart = millis();
  while (millis() - waitStart < SystemConfig::TELNET_WAIT_TIME_MS) {
      logger.handleClient(); 
      delay(50);
  }
  
  // === HARDWARE WAKE-UP DELAY ===
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
  global_imuAlive = (imuRetries < SystemConfig::IMU_MAX_RETRIES);

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  bool isColdBoot = (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED);
  
  // Initialize the Brain with the boot state!
  brain.init(isColdBoot);
  
  logger.println("Mister Mischief V1 Booting...");
  delay(1000); 

// === TASK CREATION ===
  xTaskCreatePinnedToCore(
    SensorTask, "SensorTask", 
    SystemConfig::TASK_STACK_SIZE, NULL, SystemConfig::SENSOR_TASK_PRIORITY, 
    &SensorTaskHandle, SystemConfig::SENSOR_TASK_CORE_AFFINITY
  );

  xTaskCreatePinnedToCore(
    ControlLoopTask, "ControlLoopTask", 
    SystemConfig::TASK_STACK_SIZE, NULL, SystemConfig::CONTROL_TASK_PRIORITY, 
    &ControlLoopTaskHandle, SystemConfig::CONTROL_TASK_CORE_AFFINITY
  );
}

void loop() { vTaskDelete(NULL); }