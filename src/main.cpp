#include <Arduino.h>
#include "config/PinConfig.h"         // The Blueprint
#include "hal/HCSR04_Sonar.h"         // The Sonar Factory
#include "hal/XY160D_MotorDriver.h"   // The Motor Factory
#include "objectproviders/PIDPurposeProfileFactory.h" // The PID Factory

// ==========================================
// GLOBAL HARDWARE OBJECTS
// ==========================================
HCSR04_Sonar frontSonar(HardwarePins::PIN_SONAR_TRIG, HardwarePins::PIN_SONAR_ECHO);
XY160D_MotorDriver motors;
PIDController headingHoldPID = PIDPurposeProfileFactory::createHeadingHoldPID();

// ==========================================
// INTER-CORE COMMUNICATION (THE BRIDGE)
// ==========================================
// "volatile" warns the compiler that Core 0 and Core 1 are both touching this.
volatile float global_frontDistanceCM = -1.0;

// ==========================================
// TASK HANDLES
// ==========================================
TaskHandle_t SensorTaskHandle;
TaskHandle_t MotorTaskHandle;

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
void MotorTask(void *pvParameters) {
  for (;;) {
    // 1. Check global safety variables
    // If distance is greater than 0 (valid reading) AND less than 20cm (too close!)
    if (global_frontDistanceCM > 0 && global_frontDistanceCM < 20.0) {
        
        // UNSAFE -> Cut PWM immediately
        motors.stop();
        Serial.println("OBSTACLE DETECTED! Emergency Stop.");
        
    } else {
        
        // SAFE -> Apply PWM to tracks (50% speed forward)
        motors.drive(50, 50);
        
    }
    
    // Run this motor check every 20ms
    vTaskDelay(pdMS_TO_TICKS(20));
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
    MotorTask,
    "MotorTask",
    4096,
    NULL,
    1,
    &MotorTaskHandle,
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