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
#include "behaviours/Mode_Teleop.h"
#include "core/BehaviourEngine.h"
#include "core/KinematicsEngine.h"
#include "utils/RadioManager.h"
#include "utils/RemoteLogger.h"
#include "config/CommandRegistry.h" 
#include "core/CommandProcessor.h"
#include "config/SystemConfig.h"
#include "core/RobotState.h"
#include "core/GlobalDataBus.h" // <--- THE NEW INCLUDE
#include "comms/telemetry/TelemetrySinks.h"
#include "comms/telemetry/TelemetryStreamer.h"

RemoteLogger logger(SystemConfig::WEBSOCKET_PORT);

// --- 2. ADD THE SINK INSTANTIATIONS HERE ---
USBSink usbSink;
// We pass references from the logger so the WebSocketSink knows the server state
WebSocketSink wifiSink(logger.getServer(), logger.getClientCount(), logger.getLastConnectTime());

// ==========================================
// SMART TELEMETRY STREAMER
// ==========================================
Comms::TelemetryStreamer telemetryRouter;

// ==========================================
// INTER-CORE COMMUNICATION (THE BRIDGE)
// ==========================================
volatile SystemMode GLOBAL_MODE = SystemMode::BOOTING;

// Instantiate the global memory bank!
volatile GlobalDataBank CurrentRobotData = {};

// Instantiate the low level Hardware Command Bus for sending commands from the Brain to the HAL without direct hardware access!
volatile HardwareCommandBus HardwareCommands = {}; // FIX: Added the extra falses for Accel and Mag!

// Teleoperation memory for Phone BLE or radio Control
volatile TeleopCommandBus TeleopCommands = {};

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
Mode_Teleop teleopMode(&kinematics);

// ==========================================
// MODE SWITCHER (The Brain)
// ==========================================
// Note: It still takes the hardware pointers right now, but we will remove them in the next step!
BehaviourEngine brain(&obstacleMode, &normalMode, &compassMode, &distanceMode, &dizzyMode, &sleepMode, &teleopMode); // Added teleopMode to the constructor
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
        if (CurrentRobotData.health.hardwareBitmask & Comms::HealthBit::IMU_OK) {
            FusedAngles currentAngles = imu->getAngles();
            
            // C++ volatile struct fix: We must assign the fields manually!
            CurrentRobotData.physics.imuAngles.yaw = currentAngles.yaw;
            CurrentRobotData.physics.imuAngles.pitch = currentAngles.pitch;
            CurrentRobotData.physics.imuAngles.roll = currentAngles.roll;
            CurrentRobotData.physics.imuAngles.gForce = currentAngles.gForce;
            CurrentRobotData.physics.imuAngles.hasCompass = currentAngles.hasCompass;
            CurrentRobotData.physics.imuAngles.compassHeading = currentAngles.compassHeading;
        }

        unsigned long currentTime = millis();

        // Poll the Sonar exactly every 50ms
        if (currentTime - lastSonarTime >= 50) { 
            lastSonarTime = currentTime;
            CurrentRobotData.sensors.distanceCM = frontDistanceSensor->getDistanceCM();
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

    // State tracker to detect the moment the app connects or drops
    static bool wasBleConnected = false; 

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // --- THE TELEOPERATION OVERRIDE WATCHDOG ---
        if (TeleopCommands.isConnected && !wasBleConnected) {
            wasBleConnected = true;
            SysConfig.BRAIN_ACTIVE = false;       // Shut down autonomous decision engine
            brain.changeMode(&teleopMode);     // Force the manual kinematic mixer
            logger.println("[SYSTEM] BLE Connected. Manual Override Engaged.");
        } 
        else if (!TeleopCommands.isConnected && wasBleConnected) {
            wasBleConnected = false;
            SysConfig.BRAIN_ACTIVE = true;        // Turn autonomous brain back on
            brain.changeMode(&normalMode);     // Safely recover to normal driving
            logger.println("[SYSTEM] BLE Disconnected. Autonomous Brain Resumed.");
        }
        // -------------------------------------------

        // THE ISOLATION: The Brain only runs if the Gatherer says the IMU is alive!
        if (CurrentRobotData.health.hardwareBitmask & Comms::HealthBit::IMU_OK) {
            brain.update(CurrentRobotData); 
        } else {
            motorDriver->stop(); 
        }
    }
}


// ==========================================
// TASK 3: NETWORK & TELEMETRY PUBLISHER (CORE 1 - APP CPU)
// ==========================================
void NetworkTask(void *pvParameters) {
    unsigned long lastTelemetryTime = 0;
    
    for (;;) {
        // 1. Read CLI input securely
        while (Serial.available()) {
            cliEngine.processChar(Serial.read());
        }

        // 2. Handle incoming WebSocket handshakes 
        logger.handleClient();
        
        // 3. Publish the JSON State over the air
        unsigned long currentTime = millis();
        if (currentTime - lastTelemetryTime >= SystemConfig::TELEMETRY_PING_DELAY_MS) {
            lastTelemetryTime = currentTime;
            
            if (CurrentRobotData.health.hardwareBitmask & Comms::HealthBit::IMU_OK) {
                logger.publishTelemetry(CurrentRobotData, brain.getActiveModeName(), SysConfig.BRAIN_ACTIVE);
            }
        }
        
        // Give the network stack breathing room
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

  // Register your hardware sinks (do this once in setup!)
  // telemetryRouter.registerSink(&bluetoothSink); //TO-DO: FLESH IT OUT 
  telemetryRouter.registerSink(&wifiSink);
  // telemetryRouter.registerSink(&serialSink); // Only if you want binary over USB

  // === MODEM COEXISTENCE FIX ===
  // If Bluetooth is active, the ESP32 REQUIRES Wi-Fi modem sleep to switch the antenna.
  // If only Wi-Fi is active, we can disable sleep to keep the connection rock solid.
  if (SysConfig.WIFI_ACTIVE) {
      if (SysConfig.BT_ACTIVE) {
          WiFi.setSleep(true); // Mandatory for Bluetooth Coexistence
      } else {
          WiFi.setSleep(false); // Only safe if BT is OFF
      }
  }
  pointTurnPID.setTunings(SysConfig.PID_POINT_P, SysConfig.PID_POINT_I, SysConfig.PID_POINT_D, SysConfig.PID_POINT_ILIM, SysConfig.PID_POINT_LIM);
  arcTurnPID.setTunings(SysConfig.PID_ARC_P, SysConfig.PID_ARC_I, SysConfig.PID_ARC_D, SysConfig.PID_ARC_ILIM, SysConfig.PID_ARC_LIM);
  distancePID.setTunings(SysConfig.PID_DIST_P, SysConfig.PID_DIST_I, SysConfig.PID_DIST_D, SysConfig.PID_DIST_ILIM, SysConfig.PID_DIST_LIM);
  
  imu->setFilterBeta(SysConfig.MADGWICK_FILTER_BETA);
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
  
  // Set the global state by flipping the IMU bit in GlobalDataBank
  // ADD THIS:
  if (imuRetries < SystemConfig::IMU_MAX_RETRIES) {
      // Turn the IMU bit ON
      CurrentRobotData.health.hardwareBitmask |= Comms::HealthBit::IMU_OK;
      logger.println("IMU marked as OK in Health Registry.");
  } else {
      // (Optional) Explicitly turn it OFF if it failed
      CurrentRobotData.health.hardwareBitmask &= ~Comms::HealthBit::IMU_OK;
  }
  
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  bool isColdBoot = (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED);
  
  brain.init(isColdBoot);

  // Temporary testing override
//   SysConfig.BRAIN_ACTIVE = false;       
//   brain.changeMode(&autotuneMode);   
  
  logger.println("Mister Mischief V1 Booting...");
  delay(1000); 

  // === THE NEW 4-TASK ISOLATED ARCHITECTURE ===

  // App CPU (Core 1)
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", SystemConfig::TASK_STACK_SENSOR, NULL, SystemConfig::SENSOR_TASK_PRIORITY, &SensorTaskHandle, SystemConfig::SENSOR_TASK_CORE_AFFINITY);
  xTaskCreatePinnedToCore(ControlLoopTask, "ControlLoopTask", SystemConfig::TASK_STACK_PHYSICS, NULL, SystemConfig::CONTROL_LOOP_TASK_PRIORITY, &ControlLoopTaskHandle, SystemConfig::CONTROL_LOOP_TASK_CORE_AFFINITY);
  xTaskCreatePinnedToCore(NetworkTask, "NetworkTask", SystemConfig::TASK_STACK_NETWORK, NULL, SystemConfig::NETWORK_TASK_PRIORITY, &NetworkTaskHandle, SystemConfig::NETWORK_TASK_CORE_AFFINITY); 
}

void loop() { vTaskDelete(NULL); }