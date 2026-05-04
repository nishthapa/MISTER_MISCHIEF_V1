#include <Arduino.h>
#include "config/PinConfig.h"         // The Blueprint
#include "hal/HCSR04_Sonar.h"         // The Sonar Factory
#include "hal/XY160D_MotorDriver.h"   // The Motor Factory
#include "objectproviders/PIDPurposeProfileFactory.h" // The PID Factory
#include "behaviours/Mode_BalancePlatform.h"
#include "behaviours/RobotMood.h"

// Global variables to track the boot state
RobotMood activeMood;
unsigned long coldBootTime = 0;
bool isGroggyPhase = false;

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
PIDController balancePID = PIDPurposeProfileFactory::createBalancePlatformPID();

// --- Instantiate Balancing on a Platform Mode ---
Mode_BalancePlatform balanceMode(&imu, &motors, &balancePID);

// --- The Current State Pointers ---
IRobotMode* activeMode = &balanceMode;
RobotMood activeMood = Moods::HAPPY;

void ControlLoopTask(void *pvParameters) {
    for (;;) {
        // Run the mode with the current mood!
        activeMode->update(activeMood);
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
// CORE 1: THE MOTOR CONTROLLER
// ==========================================
void ControlLoopTask(void *pvParameters) {
    for (;;) {
        // 1. Handle the Groggy Timer (ONLY if we are in a cold boot groggy phase)
        if (isGroggyPhase) {
            // Pull the configuration directly from your centralized Moods namespace
            if (millis() - coldBootTime > Moods::GROGGY_DURATION_MS) {
                Serial.println("Groggy phase over. Shaking it off!");
                activeMood = Moods::HAPPY;
                isGroggyPhase = false; // Lock this out so it doesn't trigger again
            }
        } 
        else {
            // 2. Normal Dynamic Mood Mixing (Sensor-driven)
            // If we aren't groggy, allow the Sonar to change our mood
            if (global_frontDistanceCM > 0 && global_frontDistanceCM < 15.0) {
                activeMood = Moods::ANGRY; 
            } else if (global_frontDistanceCM > 50.0) {
                // activeMood = Moods::SLEEPY;
            } else {
                activeMood = Moods::HAPPY;
            }
        }

        // 3. Apply the chosen mood to the active mode
        activeMode->update(activeMood);
        
        vTaskDelay(pdMS_TO_TICKS(10));
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