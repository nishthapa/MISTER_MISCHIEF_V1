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
Mode_ObstacleAvoidance obstacleMode(&kinematics, frontDistanceSensor, imu);
Mode_NormalDriving normalMode(imu, &kinematics); 
Mode_CompassLock compassMode(imu, &kinematics);
Mode_MaintainDistance distanceMode(frontDistanceSensor, &kinematics, &distancePID);
Mode_Dizzy dizzyMode(motorDriver);
Mode_DeepSleep sleepMode(motorDriver);
Mode_AutoTune autotuneMode(imu, &kinematics);

// ==========================================
// MODE SWITCHER (The Brain)
// ==========================================
// Note: It still takes the hardware pointers right now, but we will remove them in the next step!
BehaviourEngine brain(imu, frontDistanceSensor, &obstacleMode, &normalMode, &compassMode, &distanceMode, &dizzyMode, &sleepMode);

CommandProcessor cliEngine;

// ==========================================
// CORE 1: THE MAIN CONTROL LOOP (PHYSICS & DECISIONS)
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
// CORE 0: THE GATHERER (SENSORS & TELEMETRY)
// ==========================================
void SensorTask(void *pvParameters) {
  unsigned long lastTelemetryTime = 0;
  unsigned long lastSonarTime = 0;

  for (;;) {
    logger.handleClient();
    
    while (Serial.available()) {
        cliEngine.processChar(Serial.read()); 
    }

    unsigned long currentTime = millis();

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

    // Poll the Sonar exactly every 50ms
    if (currentTime - lastSonarTime >= 50) { 
        lastSonarTime = currentTime;
        CurrentSensorState.distanceCM = frontDistanceSensor->getDistanceCM();
    }

    // ==========================================
    // 2. TELEMETRY PRINTING (Reading from Memory!)
    // ==========================================
    if (currentTime - lastTelemetryTime >= SystemConfig::TELEMETRY_PING_DELAY_MS) {
        lastTelemetryTime = currentTime;
        
        if (Config.SERIAL_DEBUG_MASTER) {
            if (CurrentSensorState.imuAlive) {
                logger.sendTelemetryJSON("{\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,\"sonar\":%.1f,\"mode\":\"%s\",\"brain\":%s}\n", 
                              CurrentSensorState.imuAngles.yaw, 
                              CurrentSensorState.imuAngles.pitch,
                              CurrentSensorState.imuAngles.roll,
                              CurrentSensorState.distanceCM,
                              brain.getActiveModeName(), 
                              Config.BRAIN_ACTIVE ? "true" : "false");
            }
        } else {
            if (!CurrentSensorState.imuAlive && Config.BRAIN_ACTIVE) motorDriver->stop(); 
        }
    }
    
    vTaskDelay(pdMS_TO_TICKS(1)); 
  }
}

// ==========================================
// SETUP
// ==========================================
TaskHandle_t SensorTaskHandle;
TaskHandle_t ControlLoopTaskHandle;

void setup() {
  ConfigSys.init(); 
  logger.beginSerial();
  RadioManager::initRadios();
  logger.bindRadios();

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