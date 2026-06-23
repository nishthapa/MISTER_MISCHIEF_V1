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
#include "behaviours/Mode_Diagnostics.h"
#include "core/BehaviourEngine.h"
#include "core/KinematicsEngine.h"
#include "utils/RadioManager.h"
#include "utils/RemoteLogger.h"
#include "config/CommandRegistry.h" 
#include "core/CommandProcessor.h"
#include "config/SystemConfig.h"
#include "core/RobotState.h"
#include "core/GlobalDataBus.h"
#include "comms/telemetry/TelemetrySinks.h"
#include "comms/telemetry/TelemetryStreamer.h"
#include "tasks/Task_ControlLoop.h"
#include "tasks/Task_Sensor.h"
#include "tasks/Task_Network.h"
#include "utils/OTAManager.h"

// 1. Declare the context struct globally so it doesn't get destroyed after setup()
ControlLoopContext controlCtx;
SensorContext sensorCtx;
NetworkContext networkCtx;

// RemoteLogger logger(SystemConfig::WEBSOCKET_PORT);
RemoteLogger logger; // IT DOESNT OWN THE WEBSOCKET ANYMORE SO NO PORT NUMBER

// --- TELEMETRY SINK INSTANTIATIONS ---
USBSink usbSink;
DebugHexSink debugHexSink;
// We pass references from the logger so the WebSocketSink knows the server state
//WebSocketSink wifiSink(logger.getServer(), logger.getClientCount(), logger.getLastConnectTime());

// The Sink now completely owns the port and the server!
WebSocketSink wifiSink(SystemConfig::WEBSOCKET_PORT);

// Instantiate the BLE Sink
BluetoothSink bleSink;

// ==========================================
// SMART TELEMETRY ROUTER / STREAMER
// ==========================================
Comms::TelemetryStreamer telemetryRouter;

// ==========================================
// INTER-CORE COMMUNICATION (THE BRIDGE)
// ==========================================
//volatile SystemMode GLOBAL_MODE = SystemMode::BOOTING;

// Initialize the Hardware Spinlocks
portMUX_TYPE globalDataBusLock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE hardwareCmdLock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE teleopCmdLock = portMUX_INITIALIZER_UNLOCKED;

// Instantiate the Global Robot State Data bus
GlobalDataBank CurrentRobotData = {};

// Instantiate the low level Hardware Command Bus for sending commands from the Brain to the HAL without direct hardware access!
HardwareCommandBus HardwareCommands = {}; // FIX: Added the extra falses for Accel and Mag!

// Teleoperation memory for Phone BLE or radio Control
TeleopCommandBus TeleopCommands = {};

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

KinematicsEngine kinematicsEngine(&pointTurnPID, &arcTurnPID);

// ==========================================
// GLOBAL MODE OBJECTS
// ==========================================
Mode_ObstacleAvoidance obstacleMode(&kinematicsEngine); // Removed 'frontDistanceSensor' and imu
Mode_NormalDriving normalMode(&kinematicsEngine);  // Removed 'imu'
Mode_CompassLock compassMode(&kinematicsEngine); // Removed 'imu'
Mode_MaintainDistance distanceMode(&kinematicsEngine, &distancePID); // Removed 'frontDistanceSensor'
Mode_Dizzy dizzyMode(&kinematicsEngine); // standardized dependency injection from direct motor driver access level to kinematics engine level
Mode_DeepSleep sleepMode(&kinematicsEngine); // standardized dependency injection from direct motor driver access level to kinematics engine level
Mode_AutoTune autotuneMode(&kinematicsEngine); // Removed 'imu'
Mode_Teleop teleopMode(&kinematicsEngine);
Mode_Diagnostics diagnosticMode(&kinematicsEngine);


// ==========================================
// MODE SWITCHER (The Brain)
// ==========================================
// Note: It still takes the hardware pointers right now, but we will remove them in the next step!
BehaviourEngine brain(&obstacleMode, &normalMode, &compassMode, &distanceMode, &dizzyMode, &sleepMode, &teleopMode, &diagnosticMode, &autotuneMode); // Added teleopMode to the constructor
CommandProcessor cliEngine;

// ==========================================
// TASKS HAVE BEEN MOVED OUT OF main.cpp
// ==========================================

// Global Task Handles
TaskHandle_t SensorTaskHandle;
TaskHandle_t ControlLoopTaskHandle;
// TaskHandle_t TelemetryTaskHandle; // FIX: Added the 3rd task handle!
TaskHandle_t NetworkTaskHandle; // FIX: Added the 4th task handle!

void setup() {
  ConfigSys.init();

  logger.beginSerial();

  int imuRetries = 0;
  while (!imu->init() && imuRetries < SystemConfig::IMU_MAX_RETRIES) {
      logger.println("IMU failed to initialize. Rebooting I2C bus...");
      delay(SystemConfig::IMU_RETRY_DELAY_MS); 
      imuRetries++;
  }
  
  RadioManager::initRadios();
  logger.bindRadios();

  // Register your hardware sinks (do this once in setup!)
  // telemetryRouter.registerSink(&bluetoothSink); //TO-DO: FLESH IT OUT 
  if (SysConfig.WIFI_ACTIVE) {
    wifiSink.begin(); // Start the global sink we defined at the top
    telemetryRouter.registerSink(&wifiSink); // Wire it into the router!
  }

  if (SysConfig.BT_ACTIVE) {
    telemetryRouter.registerSink(&bleSink); // 2. Wire it into the router!
  }
  
  //telemetryRouter.registerSink(&usbSink); // Only if you want binary over USB
  //telemetryRouter.registerSink(&debugHexSink); // Only if you want hexadecimal output

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

  // PID tunes for Point (in-place) Turns, Arc turns and the distance hold game
  pointTurnPID.setTunings(SysConfig.PID_POINT_P, SysConfig.PID_POINT_I, SysConfig.PID_POINT_D, SysConfig.PID_POINT_ILIM, SysConfig.PID_POINT_LIM);
  arcTurnPID.setTunings(SysConfig.PID_ARC_P, SysConfig.PID_ARC_I, SysConfig.PID_ARC_D, SysConfig.PID_ARC_ILIM, SysConfig.PID_ARC_LIM);
  distancePID.setTunings(SysConfig.PID_DIST_P, SysConfig.PID_DIST_I, SysConfig.PID_DIST_D, SysConfig.PID_DIST_ILIM, SysConfig.PID_DIST_LIM);
  
  imu->setFilterBeta(SysConfig.MADGWICK_FILTER_BETA);
  logger.println("Configuration Manager loaded from permanent memory.");

  unsigned long waitStart = millis();
  while (millis() - waitStart < SystemConfig::TELNET_WAIT_TIME_MS) {
      // Let the SINK handle the background connection during boot!
      wifiSink.update(); 
      delay(50);
  }
  
  delay(SystemConfig::HARDWARE_WAKE_DELAY_MS);

  // Initialize SONAR and set HW bitmask
  if(frontDistanceSensor->init()) {
    portENTER_CRITICAL(&globalDataBusLock);
    CurrentRobotData.health.hardwareBitmask |= Comms::HealthBit::SONAR_OK;
    portEXIT_CRITICAL(&globalDataBusLock);
  }
  else {
    portENTER_CRITICAL(&globalDataBusLock);
    CurrentRobotData.health.hardwareBitmask &= ~Comms::HealthBit::SONAR_OK;
    portEXIT_CRITICAL(&globalDataBusLock);
  }

  // Initialize MOTOR DRIVER and set HW bitmask
  if(motorDriver->init()) {
    portENTER_CRITICAL(&globalDataBusLock);
    CurrentRobotData.health.hardwareBitmask |= Comms::HealthBit::MOTOR_DRIVER_OK;
    portEXIT_CRITICAL(&globalDataBusLock);
  }
  else {
    portENTER_CRITICAL(&globalDataBusLock);
    CurrentRobotData.health.hardwareBitmask &= ~Comms::HealthBit::MOTOR_DRIVER_OK;
    portEXIT_CRITICAL(&globalDataBusLock);
  }
  
  logger.println("Waking up the IMU...");
  // Set the global state for IMU and MAG by flipping the IMU bit in GlobalDataBank
  if (imuRetries < SystemConfig::IMU_MAX_RETRIES) {
      portENTER_CRITICAL(&globalDataBusLock);
      // Turn the IMU bit ON (Set to 1)
      CurrentRobotData.health.hardwareBitmask |= Comms::HealthBit::IMU_OK;
      portEXIT_CRITICAL(&globalDataBusLock);
      logger.println("[IMU] check PASSED, marked as OK in Health Registry.");
  } else {
      portENTER_CRITICAL(&globalDataBusLock);
      // Explicitly turn the IMU bit OFF (Set to 0) using bitwise AND NOT
      CurrentRobotData.health.hardwareBitmask &= ~Comms::HealthBit::IMU_OK;
      portEXIT_CRITICAL(&globalDataBusLock);
      logger.println("[IMU] check FAILED, marked as NOT-OK in Health Registry.");
    }

  // Set the compass flag based on whether HAS_COMPASS is true or not
  if(IMUConfig::HAS_COMPASS) {
    CurrentRobotData.health.hardwareBitmask |= Comms::HealthBit::MAG_OK;
  } else {
    CurrentRobotData.health.hardwareBitmask &= ~Comms::HealthBit::MAG_OK;
  }
  
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  bool isColdBoot = (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED);
  
  brain.init(isColdBoot);

  // Temporary testing override
  //SysConfig.BRAIN_ACTIVE = false;       
  //brain.changeMode(&autotuneMode);   
  
  logger.println("Mister Mischief V1 Booting...");
  delay(1000);

  bool isWifiConnected = false;

  // Listen for the ESP32 background Wi-Fi events asynchronously
  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){
    logger.print("\n[WIFI-EVENT] SUCCESS! Robot is ONLINE at IP: ");
    logger.println(WiFi.localIP().toString());

    // Flip the WiFi connected Health Bit ON
    portENTER_CRITICAL(&globalDataBusLock);
    CurrentRobotData.health.hardwareBitmask |= Comms::HealthBit::WIFI_CONNECTED;
    portEXIT_CRITICAL(&globalDataBusLock);

    // START LISTENING FOR OTA FIRMWARE UPDATES!
    OTAManager::begin();
  }, ARDUINO_EVENT_WIFI_STA_GOT_IP);

  WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info){
    // USE RAW SERIAL HERE! 
      // Do not use logger.println, otherwise it will attempt a WebSocket 
      // broadcast over a dead interface and crash the memory!
      Serial.println("\n[WIFI-EVENT] Disconnected from AP.");
      //logger.println("\n[WIFI-EVENT] Disconnected from AP.");
      
      // Flip the WiFi connected Health Bit OFF
      portENTER_CRITICAL(&globalDataBusLock);
      CurrentRobotData.health.hardwareBitmask &= ~Comms::HealthBit::WIFI_CONNECTED;
      portEXIT_CRITICAL(&globalDataBusLock);
  }, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  
  // Pack needed contexts for the new isolated ColtrolLoopTask
  controlCtx.brain = &brain;
  controlCtx.motorDriver = motorDriver;
  controlCtx.kinematics = &kinematicsEngine;
  controlCtx.distancePID = &distancePID; // for tuning Distance hold PID from the command line

  // Pack needed contexts for the new isolated SensorTask
  sensorCtx.imu = imu;
  sensorCtx.sonar = frontDistanceSensor;

  // Pack needed contexts for the new isolated NetworkTask
  networkCtx.cli = &cliEngine;
  networkCtx.router = &telemetryRouter;
  networkCtx.brain = &brain;

  // === THE NEW 3-TASK ISOLATED ARCHITECTURE ===
  BaseType_t sTask = xTaskCreatePinnedToCore(SensorTask, "SensorTask", SystemConfig::TASK_STACK_SENSOR, &sensorCtx, SystemConfig::SENSOR_TASK_PRIORITY, &SensorTaskHandle, SystemConfig::SENSOR_TASK_CORE_AFFINITY);
  BaseType_t cTask = xTaskCreatePinnedToCore(ControlLoopTask, "ControlLoopTask", SystemConfig::TASK_STACK_PHYSICS, &controlCtx, SystemConfig::CONTROL_LOOP_TASK_PRIORITY, &ControlLoopTaskHandle, SystemConfig::CONTROL_LOOP_TASK_CORE_AFFINITY);
  BaseType_t nTask = xTaskCreatePinnedToCore(NetworkTask, "NetworkTask", SystemConfig::TASK_STACK_NETWORK, &networkCtx, SystemConfig::NETWORK_TASK_PRIORITY, &NetworkTaskHandle, SystemConfig::NETWORK_TASK_CORE_AFFINITY); 

  if (sTask == pdPASS && cTask == pdPASS && nTask == pdPASS) {
      portENTER_CRITICAL(&globalDataBusLock);
      CurrentRobotData.health.hardwareBitmask |= Comms::HealthBit::FREE_RTOS_ALIVE;
      portEXIT_CRITICAL(&globalDataBusLock);
      logger.println("[OS] FreeRTOS tasks initialized successfully.");
  } else {
      logger.println("[OS] CRITICAL ERROR: FreeRTOS tasks initialization failed!");
  }

  //logger.println("==================================================");
  logger.println("FREERTOS SCHEDULER INITIATED");
  //logger.println("==================================================");
}

void loop() { vTaskDelete(NULL); }