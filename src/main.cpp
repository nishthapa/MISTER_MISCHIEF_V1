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
// CORE 0: THE PRO CLI PARSER, NOW READS
// FROM THE ROM GLOBAL PREFERENCES
// ==========================================
void executeCLICommand(String input) {
    input.trim();
    if (input.length() == 0) return;

    // 1. SPLIT THE TOKENS
    int firstSpace = input.indexOf(' ');
    String cmd = (firstSpace == -1) ? input : input.substring(0, firstSpace);
    cmd.toLowerCase(); // RULE: Commands must be lowercase (e.g., "set", "get")

    String remainder = (firstSpace == -1) ? "" : input.substring(firstSpace + 1);
    remainder.trim();
    int secondSpace = remainder.indexOf(' ');
    
    String varName = (secondSpace == -1) ? remainder : remainder.substring(0, secondSpace);
    varName.toUpperCase(); // RULE: Variables must be uppercase (e.g., "CRUISING_SPEED")

    String valStr = (secondSpace == -1) ? "" : remainder.substring(secondSpace + 1);
    valStr.trim();

    // 2. EXECUTE LOGIC
    if (cmd == "set") {
        if (varName == "") {
            logger.println("Usage: set <VARIABLE> <VALUE>");
            return;
        }

        // --- PERSONALITY VARS ---
        if (varName == "CRUISING_SPEED") { Config.CRUISING_SPEED = valStr.toFloat(); }
        else if (varName == "OBSTACLE_TRIGGER_CM") { Config.OBSTACLE_TRIGGER_CM = valStr.toFloat(); }
        else if (varName == "MAINTAIN_DIST_CM") { Config.MAINTAIN_DIST_CM = valStr.toFloat(); }

        // --- DEBUG VARS ---
        else if (varName == "SERIAL_DEBUG_MASTER") { 
            valStr.toLowerCase();
            Config.SERIAL_DEBUG_MASTER = (valStr == "on" || valStr == "true" || valStr == "1"); 
        }

        /* FOR LATER: Granular debug controls for each subsystem!
        else if (varName == "SERIAL_DEBUG_IMU") { 
            valStr.toLowerCase();
            Config.SERIAL_DEBUG_IMU = (valStr == "on" || valStr == "true" || valStr == "1"); 
        }
        else if (varName == "SERIAL_DEBUG_SONAR") { 
            valStr.toLowerCase();
            Config.SERIAL_DEBUG_SONAR = (valStr == "on" || valStr == "true" || valStr == "1"); 
        }
        else if (varName == "SERIAL_DEBUG_MOTOR_DRIVER") { 
            valStr.toLowerCase();
            Config.SERIAL_DEBUG_MOTOR_DRIVER = (valStr == "on" || valStr == "true" || valStr == "1"); 
        }*/

        // --- SYSTEM VARS ---
        else if (varName == "BRAIN_ACTIVE") { 
            valStr.toLowerCase();
            Config.BRAIN_ACTIVE = (valStr == "on" || valStr == "true" || valStr == "1"); 
        }

        else {
            logger.printf("Unknown variable: %s\n", varName.c_str());
            return;
        }

        ConfigSys.save(); // Save to permanent memory!
        logger.printf("Successfully set %s to %s\n", varName.c_str(), valStr.c_str());
    } 
    else if (cmd == "get") {
        if (varName == "CRUISING_SPEED") { logger.printf("CRUISING_SPEED = %.1f\n", Config.CRUISING_SPEED); }
        else if (varName == "OBSTACLE_TRIGGER_CM") { logger.printf("OBSTACLE_TRIGGER_CM = %.1f\n", Config.OBSTACLE_TRIGGER_CM); }
        else if (varName == "MAINTAIN_DIST_CM") { logger.printf("MAINTAIN_DIST_CM = %.1f\n", Config.MAINTAIN_DIST_CM); }

        // --- DEBUG VARS ---
        else if (varName == "SERIAL_DEBUG_MASTER") { 
            valStr.toLowerCase();
            Config.SERIAL_DEBUG_MASTER = (valStr == "on" || valStr == "true" || valStr == "1"); 
        }
        
        /* FOR LATER: Granular debug controls for each subsystem!
        else if (varName == "SERIAL_DEBUG_IMU") { 
            valStr.toLowerCase();
            Config.SERIAL_DEBUG_IMU = (valStr == "on" || valStr == "true" || valStr == "1"); 
        }
        else if (varName == "SERIAL_DEBUG_SONAR") { 
            valStr.toLowerCase();
            Config.SERIAL_DEBUG_SONAR = (valStr == "on" || valStr == "true" || valStr == "1"); 
        }
        else if (varName == "SERIAL_DEBUG_MOTOR_DRIVER") { 
            valStr.toLowerCase();
            Config.SERIAL_DEBUG_MOTOR_DRIVER = (valStr == "on" || valStr == "true" || valStr == "1"); 
        }*/

        // --- SYSTEM VARS ---
        else if (varName == "BRAIN_ACTIVE") { 
            valStr.toLowerCase();
            Config.BRAIN_ACTIVE = (valStr == "on" || valStr == "true" || valStr == "1"); 
        }

        else { logger.printf("Unknown variable: %s\n", varName.c_str()); }
    }
    else if (cmd == "reset") {
        if (varName == "ALL") {
            logger.println("WIPING NON-VOLATILE MEMORY...");
            ConfigSys.factoryReset();
            logger.println("Factory Reset Complete. Rebooting in 3 seconds...");
            delay(3000);
            ESP.restart(); // Physically reboot the robot!
        } else {
            logger.println("Usage: reset ALL");
        }
    }
    else if (cmd == "calib") { // Keeping your old hardware commands!
        if (varName == "GYRO") { imu->calibrateGyro(); }
        else if (varName == "ACCEL") { imu->calibrateAccel(); }
        else if (varName == "MAG") { imu->calibrateMag(); }
        else { logger.println("Usage: calib <GYRO|ACCEL|MAG>"); }
    }
    else {
        logger.printf("Unknown command: %s\n", cmd.c_str());
        logger.println("Available commands: set, get, reset, calib");
    }
}

// ==========================================
// CLI COMMAND HISTORY BUFFER
// ==========================================
constexpr int HISTORY_MAX = 5;
String cmdHistory[HISTORY_MAX];
int historyCount = 0;
int historyIndex = -1; // -1 means we are currently typing a new command

void SensorTask(void *pvParameters) {
  static String cliBuffer = ""; 
  
  // Two separate stopwatches!
  unsigned long lastTelemetryTime = 0;
  unsigned long lastSonarTime = 0;

  for (;;) {
    logger.handleClient();
    
    // ==========================================
    // 1. INSTANT CLI PROCESSING (Runs every 1ms)
    // ==========================================
    while (Serial.available()) {
        char c = Serial.read();
        
        // --- ANSI ESCAPE SEQUENCE DECODER ---
        if (c == 27) { 
            delay(5); 
            if (Serial.available() >= 2) {
                char bracket = Serial.read();
                char arrow = Serial.read();
                
                if (bracket == '[') {
                    if (arrow == 'A') { // UP ARROW
                        if (historyCount > 0) {
                            if (historyIndex == -1) historyIndex = historyCount - 1; 
                            else if (historyIndex > 0) historyIndex--;               
                            
                            Serial.print("\rmischief>                                          \r"); 
                            cliBuffer = cmdHistory[historyIndex];
                            Serial.print("mischief> " + cliBuffer);
                        }
                    } 
                    else if (arrow == 'B') { // DOWN ARROW
                        if (historyIndex != -1) {
                            if (historyIndex < historyCount - 1) {
                                historyIndex++;
                                cliBuffer = cmdHistory[historyIndex];
                            } else {
                                historyIndex = -1; 
                                cliBuffer = "";
                            }
                            Serial.print("\rmischief>                                          \r"); 
                            Serial.print("mischief> " + cliBuffer);
                        }
                    }
                }
            }
            continue; 
        }

        // --- STANDARD TYPING ---
        if (c == '\r') continue; 
        
        if (c == '\b' || c == 127) { 
            if (cliBuffer.length() > 0) {
                cliBuffer.remove(cliBuffer.length() - 1);
                Serial.print("\b \b"); 
            }
            continue;
        }

        if (c == '\n') {
            Serial.println(); 
            if (cliBuffer.length() > 0) { 
                
                // SAVE COMMAND TO HISTORY
                if (historyCount == 0 || cmdHistory[historyCount - 1] != cliBuffer) {
                    if (historyCount < HISTORY_MAX) {
                        cmdHistory[historyCount++] = cliBuffer;
                    } else {
                        for (int i = 0; i < HISTORY_MAX - 1; i++) cmdHistory[i] = cmdHistory[i + 1];
                        cmdHistory[HISTORY_MAX - 1] = cliBuffer;
                    }
                }
                historyIndex = -1; 

                executeCLICommand(cliBuffer);
                cliBuffer = ""; 
            } else {
                if (Config.SERIAL_DEBUG_MASTER) {
                    Config.SERIAL_DEBUG_MASTER = false;
                    ConfigSys.save();
                    logger.println("[SYSTEM] Telemetry Paused! Type 'set SERIAL_DEBUG_MASTER on' to resume.");
                }
            }
            Serial.print("mischief> "); 
        } 
        else { 
            cliBuffer += c; 
            Serial.print(c); 
        }
    }

    unsigned long currentTime = millis();

    // ==========================================
    // 2. SONAR PHYSICS (Strictly limited to 50ms = 20Hz)
    // ==========================================
    // Allowing 50ms ensures all sound waves from the previous ping have safely died out!
    if (currentTime - lastSonarTime >= SystemConfig::TELEMETRY_PING_DELAY_MS) {
        lastSonarTime = currentTime;
        global_frontDistanceCM = frontDistanceSensor->getDistanceCM();
    }

    // ==========================================
    // 3. TELEMETRY PRINTING (Strictly limited to 50ms)
    // ==========================================
    if (currentTime - lastTelemetryTime >= SystemConfig::TELEMETRY_PING_DELAY_MS) {
        lastTelemetryTime = currentTime;
        
        if (Config.SERIAL_DEBUG_MASTER) {
            if (Config.SERIAL_DEBUG_SONAR) {
                logger.printf("[SONAR] Distance: %.1f cm | \n", global_frontDistanceCM);
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