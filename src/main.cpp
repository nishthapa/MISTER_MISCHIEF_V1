#include <Arduino.h>

#include "config/PinConfig.h"         // The Blueprint
//#include "hal/HCSR04_Sonar.h"         // The Sonar Factory
//#include "hal/XY160D_MotorDriver.h"   // The Motor Factory

#include "config/PersonalityConfig.h"
#include "objectproviders/PIDPurposeProfileFactory.h" // The PID Factory
#include "behaviours/Mode_NormalDriving.h"
#include "behaviours/Mode_CompassLock.h"
#include "behaviours/Mode_MaintainDistance.h"
#include "behaviours/Mode_ObstacleAvoidance.h"
#include "behaviours/Mode_Dizzy.h"
#include "behaviours/Mode_DeepSleep.h"
#include "behaviours/RobotMood.h"

// Global variables to track the boot state
RobotMood activeMood;
unsigned long coldBootTime = 0;
bool isGroggyPhase = false;
float frustrationLevel = 0.0f;

Mode_ObstacleAvoidance obstacleMode(&motors, &frontSonar, &imu, &compassPID);
// ...
float lastDistance = -1.0f; // Track distance for delta calculations

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
MPU6050_IMU imu(
    HardwarePins::PIN_I2C_SDA, 
    HardwarePins::PIN_I2C_SCL, 
    HardwarePins::PIN_IMU_INT, 
    0x68
);

// The PID Profiles
PIDController headingHoldPID = PIDPurposeProfileFactory::createHeadingHoldPID();
PIDController compassPID = PIDPurposeProfileFactory::createCompassLockPID();
PIDController distancePID = PIDPurposeProfileFactory::createDistanceHoldPID();

// --- Instantiate Balancing on a Platform Mode ---
Mode_NormalDriving normalMode(&motors); // The new default mode
Mode_CompassLock compassMode(&imu, &motors, &compassPID);
Mode_MaintainDistance distanceMode(&frontSonar, &motors, &distancePID);
Mode_Dizzy dizzyMode(&motors);
Mode_DeepSleep sleepMode(&motors);

// --- The Current State Pointers ---
IRobotMode* activeMode = &compassMode;
RobotMood activeMood = Moods::HAPPY;

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
        FusedAngles currentAngles = imu.getAngles();
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
                Serial.println("Groggy phase over. Mask dropped!");
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
// INTER-CORE COMMUNICATION (THE BRIDGE)
// ==========================================
// "volatile" warns the compiler that Core 0 and Core 1 are both touching this.
volatile float global_frontDistanceCM = -1.0;

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
    // 1. Ping the HC-SR04
    global_frontDistanceCM = frontSonar.getDistanceCM();
    
    // Print the distance to the Serial Monitor so you can verify it
    Serial.printf("Sonar Distance: %.1f cm\n", global_frontDistanceCM);
    
    // FreeRTOS delay: Wait 50ms before pinging again to prevent acoustic echoes from overlapping
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}

// ==========================================
// SETUP: INITIALIZE HARDWARE & SCHEDULER
// ==========================================
void setup() {
  Serial.begin(115200);
  
  // 1. WAKE UP THE HARDWARE BEFORE STARTING CORES
  frontSonar.init();
  motors.init();
  imu.init();

  // Ask the hardware: "Why did we turn on?"
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED) {
        // This means someone physically flipped the power switch (Cold Boot)
        Serial.println("Cold Boot Detected. Waking up groggy...");
        activeMood = Moods::GROGGY;
        isGroggyPhase = true;
        coldBootTime = millis();
    } else {
        // This means we woke up from Deep Sleep (e.g., IMU interrupt or BLE)
        Serial.println("Woke from Deep Sleep! Ready to greet!");
        activeMood = Moods::HAPPY; 
        isGroggyPhase = false; // Skip the groggy phase entirely
    }
  
  Serial.println("Mister Mischief V1 Booting...");
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