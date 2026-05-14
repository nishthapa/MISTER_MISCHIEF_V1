#include <Arduino.h>

#include "config/PinConfig.h"         // The Blueprint
//#include "hal/HCSR04_Sonar.h"         // The Sonar Factory
//#include "hal/XY160D_MotorDriver.h"   // The Motor Factory
#include "hal/IMUFactory.h"          // The IMU Factory

#include "config/PersonalityConfig.h"
#include "objectproviders/PIDPurposeProfileFactory.h" // The PID Factory
#include "behaviours/Mode_NormalDriving.h"
#include "behaviours/Mode_CompassLock.h"
#include "behaviours/Mode_MaintainDistance.h"
#include "behaviours/Mode_ObstacleAvoidance.h"
#include "behaviours/Mode_Dizzy.h"
#include "behaviours/Mode_DeepSleep.h"
#include "behaviours/RobotMood.h"

#include "config/WiFiConfig.h"
#include "utils/RadioManager.h"
#include "utils/RemoteLogger.h"
#include "config/DebugConfig.h" // only to check if USB debugging is enabled for the boot report

RemoteLogger logger(NetworkConfig::TELNET_PORT); // For remote telemetry / serial monitor over WiFi (Telnet)

// Global variables to track the boot state
RobotMood activeMood;
unsigned long coldBootTime = 0;
bool isGroggyPhase = false;
float frustrationLevel = 0.0f;

float lastDistance = -1.0f; // Track distance for delta calculations

// ==========================================
// INTER-CORE COMMUNICATION (THE BRIDGE)
// ==========================================
// "volatile" warns the compiler that Core 0 and Core 1 are both touching this.
volatile float global_frontDistanceCM = -1.0;
volatile float global_yaw = 0.0;
volatile float global_pitch = 0.0;
volatile float global_roll = 0.0;
volatile bool global_imuAlive = false; // to check if the imu has booted up on battery power or if it's still initializing

// ==========================================
// GLOBAL HARDWARE OBJECTS
// ==========================================
HCSR04_Sonar frontSonar(HardwarePins::PIN_SONAR_TRIG, HardwarePins::PIN_SONAR_ECHO);

// The newly refactored, fully injected Motor Driver Factory
XY160D_MotorDriver motors(
    HardwarePins::PIN_MOTOR_LEFT_FWD,
    HardwarePins::PIN_MOTOR_LEFT_REV,
    HardwarePins::PIN_MOTOR_RIGHT_FWD,
    HardwarePins::PIN_MOTOR_RIGHT_REV
);

// Instantiate the IMU (The Inner Ear)
I_IMU* imu = IMUFactory::createIMU();

// The PID Profiles

PIDController headingHoldPID = PIDPurposeProfileFactory::createHeadingHoldPID();
PIDController compassPID = PIDPurposeProfileFactory::createCompassLockPID();
PIDController distancePID = PIDPurposeProfileFactory::createDistanceHoldPID();
PIDController obstacleAvoidancePID = PIDPurposeProfileFactory::createObstacleAvoidanceNewPathScanSweepPID(); // Using a dedicated PID for obstacle avoidance

Mode_ObstacleAvoidance obstacleMode(&motors, &frontSonar, imu, &obstacleAvoidancePID);
Mode_NormalDriving normalMode(&motors); // The new default mode
Mode_CompassLock compassMode(imu, &motors, &compassPID); // --- Instantiate Balancing on a Platform Mode ---
Mode_MaintainDistance distanceMode(&frontSonar, &motors, &distancePID);
Mode_Dizzy dizzyMode(&motors);
Mode_DeepSleep sleepMode(&motors);

// --- The Current State Pointers ---
IRobotMode* activeMode = &compassMode;
//activeMood = Moods::HAPPY;

// State tracking
unsigned long dizzyStartTime = 0;
bool isDizzy = false;
FusedAngles lastAngles = {0, 0, 0};

// ==========================================
// CORE 1: THE MAIN CONTROL LOOP
// ==========================================
void ControlLoopTask(void *pvParameters) { 
    IRobotMode* previousMode = nullptr; // To track mode changes for onEnter/onExit
    for (;;) {
        FusedAngles currentAngles = imu->getAngles();
        // FusedAngles currentAngles = {0, 0, 0}; // Dummy angles for testing without the IMU
        
        // UPDATE THE BRIDGE FOR TELEMETRY
        global_yaw = currentAngles.yaw;
        global_pitch = currentAngles.pitch;
        global_roll = currentAngles.roll;

        float distance = frontSonar.getDistanceCM();

        // ==========================================
        // --- 1. SENSOR DERIVATIVES (The Triggers) ---
        // ==========================================
        
        // Calculate the Distance Delta (Sudden pop-in vs Gradual wall)
        float distanceDelta = 0.0f;
        // Only calculate if we have a valid previous reading to avoid boot-up spikes
        if (distance != lastDistance && lastDistance > 0.0f) {
            distanceDelta = distance - lastDistance;
        }
        lastDistance = distance;
             
        auto getShortestAngleDelta = [](float current, float previous) {
            float delta = current - previous;
            if (delta > 180.0f) delta -= 360.0f;
            else if (delta < -180.0f) delta += 360.0f;
            return abs(delta);
        };

        float yawRate = getShortestAngleDelta(currentAngles.yaw, lastAngles.yaw) / 0.01f;
        float pitchRate = getShortestAngleDelta(currentAngles.pitch, lastAngles.pitch) / 0.01f;
        float rollRate = getShortestAngleDelta(currentAngles.roll, lastAngles.roll) / 0.01f;
        lastAngles = currentAngles;

        // ==========================================
        // --- 2. THE DECISION TREE (Select the Mode) ---
        // ==========================================
        
        // PRIORITY 1: EMERGENCY & SPATIAL AWARENESS (The unbreakable rule)
        
        // Lock 1A: If we are currently performing an escape scan, DO NOT INTERRUPT
        if (activeMode == &obstacleMode && !obstacleMode.isSequenceComplete()) {
            activeMode = &obstacleMode;
        }
        // Lock 1B: Look for new obstacles
        else if (distance > 0 && distance < PersonalityConfig::OBSTACLE_TRIGGER_DISTANCE_CM) {
            
            // Scenario A: We are ALREADY playing with the hand. Don't interrupt the game!
            if (activeMode == &distanceMode) {
                activeMode = &distanceMode;
            }
            // Scenario B: We were driving, and an object SUDDENLY appeared (Dropped > 15cm in an instant)
            else if (distanceDelta < -15.0f) {
                activeMode = &distanceMode; // It's a hand! Play with it!
                isDizzy = false; 
            }
            // Scenario C: We were driving, and GRADUALLY approached an object
            else {
                activeMode = &obstacleMode; // It's a wall! Scan and escape!
                isDizzy = false; 
            }
        } 
        
        // PRIORITY 2: Did we just get spun wildly on any axis?
        else if ((yawRate > 300.0f || pitchRate > 300.0f || rollRate > 300.0f) && !isDizzy) {
            isDizzy = true;
            dizzyStartTime = millis();
            activeMode = &dizzyMode;
        }
        
        // Handle Dizzy Timer
        if (isDizzy && activeMode == &dizzyMode) {
            if (millis() - dizzyStartTime > 10000) {
                isDizzy = false; 
                activeMode = &normalMode; 
            }
        } 
        // PRIORITY 3: Did someone pick me up? (Compass Lock)
        else if (abs(currentAngles.pitch) > 15.0f || abs(currentAngles.roll) > 15.0f) {
            activeMode = &compassMode;
        }
        // PRIORITY 4: Default Behavior
        else if (!isDizzy) {
            activeMode = &normalMode; 
        }

        // ==========================================
        // --- 3. THE MOOD ENGINE (Select the Mood) ---
        // ==========================================
        
        // 3A. ALWAYS run the Frustration Engine in the background
        if (activeMode == &distanceMode) {
            frustrationLevel += PersonalityConfig::FRUSTRATION_HEATUP_RATE; 
        } else {
            frustrationLevel -= PersonalityConfig::FRUSTRATION_COOLDOWN_RATE; 
        }

        // Clamp the math so it doesn't break
        if (frustrationLevel < 0.0f) frustrationLevel = 0.0f;
        if (frustrationLevel > PersonalityConfig::DISTANCE_HOLD_FRUSTRATION_LIMIT + 50.0f) {
            frustrationLevel = PersonalityConfig::DISTANCE_HOLD_FRUSTRATION_LIMIT + 50.0f;
        }

        // Determine the Base Emotion
        if (frustrationLevel >= PersonalityConfig::DISTANCE_HOLD_FRUSTRATION_LIMIT) {
            activeMood = Moods::ANGRY;
        } else {
            activeMood = Moods::HAPPY;
        }

        // 3B. The Physical Override (Grogginess masks the underlying emotion)
        if (isGroggyPhase) {
            if (millis() - coldBootTime > Moods::GROGGY_DURATION_MS) {
                logger.println("Groggy phase over. Mask dropped!");
                isGroggyPhase = false; // Lock this out
                // Notice we DON'T force activeMood = HAPPY here anymore!
                // If you teased him while he was waking up, his base emotion is already ANGRY!
            } else {
                // He is still groggy. Override whatever he is feeling and force the sluggish physics.
                activeMood = Moods::GROGGY; 
            }
        }

        // --- 4. APPLY BLUETOOTH / ABSENCE LOGIC (Pseudo-code for later) ---
        // if (hoursSinceUserLastSeen > 2) {
        //     activeMode = &sleepMode; // Will trigger onEnter() and kill the CPU
        // }

        // ==========================================
        // --- 5. THE TRANSITION MANAGER (NEW!) ---
        // ==========================================
        
        // If the decision tree picked a NEW mode this tick...
        if (activeMode != previousMode) {
            
            // 1. Clean up the old mode (if one existed)
            if (previousMode != nullptr) {
                previousMode->onExit();
            }
            
            // 2. Initialize the new mode
            if (activeMode != nullptr) {
                activeMode->onEnter();
            }
            
            // 3. Update our memory so we don't trigger this again next tick
            previousMode = activeMode;
        }

        // ==========================================
        // --- 6. EXECUTE ---
        // ==========================================
        
        // Feed the carefully selected mood into the active mode
        if (activeMode != nullptr) {
            activeMode->update(activeMood);
        }

        // Run this physics loop at exactly 100Hz
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==========================================
// TASK HANDLES
// ==========================================
TaskHandle_t SensorTaskHandle;
TaskHandle_t ControlLoopTaskHandle;

// ==========================================
// CORE 0: THE SENSOR WATCHDOG
// ==========================================
void SensorTask(void *pvParameters) {
  for (;;) {
    // Keep the TCP socket alive and check for PC connections
    logger.handleClient();

    // 1. Ping the HC-SR04
    global_frontDistanceCM = frontSonar.getDistanceCM();
    
    // Print the distance to the Serial Monitor so you can verify it
    // logger.printf("Sonar Distance: %.1f cm\n", global_frontDistanceCM);
    // logger.printf("Sonar Distance: %.1f cm\n", global_frontDistanceCM);

    // 2. TRANSMIT TELEMETRY
    // Print the Distance AND the IMU Angles!
    if (global_imuAlive) {
        logger.printf("Sonar: %.1f cm | Y: %5.1f | P: %5.1f | R: %5.1f\n", 
                      global_frontDistanceCM, global_yaw, global_pitch, global_roll);
    } else {
        logger.printf("Sonar: %.1f cm | IMU: DEAD (ZOMBIE STATE)\n", 
                      global_frontDistanceCM);
    }
    
    // FreeRTOS delay: Wait 50ms before pinging again to prevent acoustic echoes from overlapping
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}

// ==========================================
// SETUP: INITIALIZE HARDWARE & SCHEDULER
// ==========================================
void setup() {
  // 1. BOOT THE LOGGER CORE (Starts Serial if requested)
  logger.beginSerial();

  // 2. BOOT RADIOS 
  RadioManager::initRadios();
  
  // 3. BIND TELEMETRY (Opens Telnet/BLE)
  logger.bindRadios();

  // ==========================================
  // THE TELNET WAITING ROOM (The Race Condition Fix)
  // ==========================================
  // Force the ESP32 to pause for 8 seconds and actively listen for your 
  // Putty connection before it boots the MPU6050 and runs the Radar Sweep!
  unsigned long waitStart = millis();
  while (millis() - waitStart < 8000) {
      logger.handleClient(); 
      delay(50);
  }
  // ==========================================

  // === THE FIX: HARDWARE WAKE-UP DELAY ===
  // Give the Mini560 power bus and the MPU6050 silicon half a second to fully stabilize 
  // before we start interrogating them over I2C.
  delay(500);
  
  // 4. WAKE UP THE HARDWARE
  frontSonar.init();
  motors.init();

  // === THE BULLETPROOF BOOT LOOP ===
  logger.println("Waking up the IMU...");
  int imuRetries = 0;
  while (!imu->init() && imuRetries < 5) {
      logger.println("IMU failed to initialize. Rebooting I2C bus in 1 second...");
      delay(1000); // Wait 1 second and try again
      imuRetries++;
  }
  
  global_imuAlive = (imuRetries < 5); // Remember if it survived or not

    // Ask the hardware: "Why did we turn on?"
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
        // This means someone physically flipped the power switch (Cold Boot)
        logger.println("Cold Boot Detected. Waking up groggy...");
        //logger.println("Cold Boot Detected. Waking up groggy...");
        activeMood = Moods::GROGGY;
        isGroggyPhase = true;
        coldBootTime = millis();
    } else {
        // This means we woke up from Deep Sleep (e.g., IMU interrupt or BLE)
        logger.println("Woke from Deep Sleep! Ready to greet!");
        //logger.println("Woke from Deep Sleep! Ready to greet!");
        activeMood = Moods::HAPPY; 
        isGroggyPhase = false; // Skip the groggy phase entirely
    }
  
  logger.println("Mister Mischief V1 Booting...");
  //logger.println("Mister Mischief V1 Booting...");
  delay(1000); // Give yourself a second to open the Serial Monitor

  // 2. Pin Tasks to Cores
  xTaskCreatePinnedToCore(
    SensorTask,         // Function to implement the task
    "SensorTask",       // Name of the task
    4096,               // Stack size in words
    NULL,               // Task input parameter
    1,                  // Priority of the task
    &SensorTaskHandle,  // Task handle
    0                   // Pin task to Core 0
  );

  xTaskCreatePinnedToCore(
    ControlLoopTask,
    "ControlLoopTask",
    4096,
    NULL,
    1,
    &ControlLoopTaskHandle,
    1                   // Pin task to Core 1
  );
}

// ==========================================
// STANDARD LOOP (ABANDONED)
// ==========================================
void loop() {
  // We leave this empty. FreeRTOS is handling everything in the background!
  vTaskDelete(NULL); 
}