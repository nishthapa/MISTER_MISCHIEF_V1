#include <Arduino.h>

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
// CORE 1: THE MAIN CONTROL LOOP
// ==========================================
void ControlLoopTask(void *pvParameters) { 
    const TickType_t xFrequency = pdMS_TO_TICKS(SystemConfig::MAIN_LOOP_TICK_RATE_MS);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Run the entire physics and mood engine in one line!
        brain.update();

        // Update Telemetry Bridge
        FusedAngles currentAngles = imu->getAngles();
        global_yaw = currentAngles.yaw;
        global_pitch = currentAngles.pitch;
        global_roll = currentAngles.roll;
    }
}

// ==========================================
// CORE 0: THE SENSOR WATCHDOG
// ==========================================
void executeCLICommand(String cmd) {
    CLI::CommandCode code = CLI::parseCommand(cmd);
    switch (code) {
        case CLI::CommandCode::CALIB_GYRO: imu->calibrateGyro(); break;
        case CLI::CommandCode::CALIB_ACCEL: imu->calibrateAccel(); break;
        case CLI::CommandCode::CALIB_MAG: imu->calibrateMag(); break;
        case CLI::CommandCode::CALIB_SONAR: logger.println("[CLI] Not implemented."); break;
        default: CLI::printHelpMenu(logger); break;
    }
}

void SensorTask(void *pvParameters) {
  static String cliBuffer = ""; 

  for (;;) {
    logger.handleClient();
    while (Serial.available()) {
        char incomingChar = Serial.read();
        if (incomingChar == '\n' || incomingChar == '\r') {
            if (cliBuffer.length() > 0) { executeCLICommand(cliBuffer); cliBuffer = ""; }
        } else { cliBuffer += incomingChar; }
    }

    global_frontDistanceCM = frontDistanceSensor->getDistanceCM();

    if (global_imuAlive) {
        // Look how cleanly we can ask the brain for the current mode/mood!
        logger.printf("SONAR: %.1f cm | Y: %5.1f | P: %5.1f | R: %5.1f | MODE: %s | MOOD: %s\n", 
                      global_frontDistanceCM, global_yaw, global_pitch, global_roll, 
                      brain.getActiveModeName(), brain.getActiveMoodName());
    } else {
        logger.printf("SONAR: %.1f cm | IMU: N/A | MODE: %s | MOOD: %s\n", 
                      global_frontDistanceCM, brain.getActiveModeName(), brain.getActiveMoodName());
                      motorDriver->stop();
    }
    vTaskDelay(pdMS_TO_TICKS(SystemConfig::TELEMETRY_PING_DELAY_MS)); 
  }
}

// ==========================================
// SETUP: INITIALIZE HARDWARE & SCHEDULER
// ==========================================
TaskHandle_t SensorTaskHandle;
TaskHandle_t ControlLoopTaskHandle;

void setup() {
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