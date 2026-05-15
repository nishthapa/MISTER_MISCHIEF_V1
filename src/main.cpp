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
#include "config/CommandRegistry.h" // to run our terminal commands

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

    // === THE FIX 9: GLOBAL MEMORY ===
    // Moved these to the top so they survive between ticks AND can be wiped by Dizzy Mode!
    static unsigned long settlingStartTime = 0;
    static bool settlingTimerActive = false;
    static unsigned long pickupStartTime = 0;
    static bool pickupTimerActive = false;
    static bool isHandling = false;
    static bool hasExperiencedLift = false; // NEW: The Gravity Latch!

    // === THE SOFTWARE METRONOME SETUP ===
    // 10ms = 100Hz (The professional standard for flight controllers)
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Lock the loop to exactly 100Hz. 
        // This ensures dt in the Madgwick filter is always perfectly accurate!
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

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
        
        // Calculate the Distance Delta
        float distanceDelta = 0.0f;
        if (distance != lastDistance && lastDistance > 0.0f) {
            distanceDelta = distance - lastDistance;
        }
        lastDistance = distance;
             
        auto getShortestAngleDelta = [](float current, float previous) {
            float delta = current - previous;
            if (delta > 180.0f) delta -= 360.0f;
            else if (delta < -180.0f) delta += 360.0f;
            
            // THE FIX 1: Do not take the absolute value here! 
            // Return the true signed (+ or -) movement so vibrations cancel out.
            return delta;
        };

        // --- NEW KINETIC ENERGY "DIZZY BAR" LOGIC ---
        // 1. Calculate the Raw Kinetic Energy (Absolute Speed)
        // We take the abs() here because we want to measure the total energy spent shaking him!
        float rawYawEnergy = abs(getShortestAngleDelta(currentAngles.yaw, lastAngles.yaw)) / 0.01f;
        float rawPitchEnergy = abs(getShortestAngleDelta(currentAngles.pitch, lastAngles.pitch)) / 0.01f;
        float rawRollEnergy = abs(getShortestAngleDelta(currentAngles.roll, lastAngles.roll)) / 0.01f;
        
        // THE FIX 7.1: Continuous Energy Smoothing for Floor Detection
        // Calculate total energy and run it through an EMA to ignore 1-frame sensor noise spikes!
        float totalRawEnergy = rawYawEnergy + rawPitchEnergy + rawRollEnergy;
        static float smoothedTotalEnergy = 0.0f;
        
        // THE FIX 8: Increased alpha to 0.10f so it detects hand tremors faster and doesn't over-smooth!
        smoothedTotalEnergy = (0.10f * totalRawEnergy) + (0.90f * smoothedTotalEnergy);

        // --- THE FIX 10: GRAVITY VECTOR PRE-CHECK! ---
        // Slowly lowering him is indistinguishable from holding him steady, 
        // UNLESS we look at the raw acceleration! 
        // Pull the live physical G-Force from the upgraded IMU struct!
        float currentGForce = currentAngles.gForce;
        
        // A standard 1G environment reads ~1.0. Lifting him sharply will spike this > 1.2 or < 0.8
        if (currentGForce > 1.2f || currentGForce < 0.8f || totalRawEnergy > 300.0f) {
            hasExperiencedLift = true; // The physical proof he was picked up!
        }

        // 2. THE KINETIC ENERGY ACCUMULATOR
        // THE FIX 2: Replaced the Transport Freeze with an "Energy Deadband"!
        // Normal carrying micro-movements (< 30) generate ZERO dizzy charge. 
        // Violent rotations (> 30) WILL charge the bar, EVEN IF he is in Compass Lock!
        // This guarantees he can always get dizzy if you shake him hard enough.
        float effectiveYawEnergy = (rawYawEnergy > 30.0f) ? rawYawEnergy : 0.0f;
        float effectivePitchEnergy = (rawPitchEnergy > 30.0f) ? rawPitchEnergy : 0.0f;
        float effectiveRollEnergy = (rawRollEnergy > 30.0f) ? rawRollEnergy : 0.0f;

        static float dizzyBarYaw = 0.0f;
        static float dizzyBarPitch = 0.0f;
        static float dizzyBarRoll = 0.0f;

        // At 100Hz, a 0.002f multiplier requires over 4 seconds of continuous shaking to trigger!
        dizzyBarYaw = (0.002f * effectiveYawEnergy) + (0.998f * dizzyBarYaw);
        dizzyBarPitch = (0.002f * effectivePitchEnergy) + (0.998f * dizzyBarPitch);
        dizzyBarRoll = (0.002f * effectiveRollEnergy) + (0.998f * dizzyBarRoll);

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
        // 💥 NEW LOCK 1C: Hysteresis for Distance Game (Prevent fluttering at the trigger edge)
        // If the hand backs up to 26cm (trigger is 25cm), don't instantly drop the game.
        // Wait until the hand is fully pulled away (> 40cm) before returning to normal!
        else if (activeMode == &distanceMode && distance > 0 && distance < (PersonalityConfig::OBSTACLE_TRIGGER_DISTANCE_CM + 15.0f)) {
            activeMode = &distanceMode; 
        }

        // PRIORITY 2: THE DIZZY TRAP (Re-promoted!)
        // THE FIX 3: Set threshold to 150. Combined with the 4-second slow charge and deadband, it's perfect.
        // If the Dizzy Bar hits 150, trigger dizzy mode and override Compass Lock!
        else if ((dizzyBarYaw > 150.0f || dizzyBarPitch > 150.0f || dizzyBarRoll > 150.0f) && !isDizzy) {
            isDizzy = true;
            dizzyStartTime = millis();
            activeMode = &dizzyMode;
            
            // Instantly drain the energy bars so he doesn't re-trigger immediately after waking up!
            dizzyBarYaw = 0.0f; dizzyBarPitch = 0.0f; dizzyBarRoll = 0.0f;
        }
        
        // Handle Dizzy Timer
        // The Untrapped Timer
        // Let the timer run independently so it doesn't freeze if picked up.
        else if (isDizzy) {
            if (millis() - dizzyStartTime > 10000) {
                isDizzy = false; 
                
                // THE FIX 9: The Ghost Latch Wipe!
                // We MUST clear these out. If we don't, motor vibrations from Normal Mode 
                // will falsely trigger the Compass Entry timer.
                isHandling = false;
                hasExperiencedLift = false; 
                pickupTimerActive = false;
                settlingTimerActive = false;
                
                // Only return to normal if he isn't currently being held by a human
                if (activeMode == &dizzyMode) {
                    activeMode = &normalMode; 
                }
            } else {
                activeMode = &dizzyMode; // Force him back to dizzy if Priority 1 temporarily interrupted it!
            }
        }

        // PRIORITY 3: PHYSICAL HANDLING (Compass Lock)
        // This only runs if he IS NOT dizzy. 
        else {
            // THE FIX 5 & 10: The "Handling" Latch with GRAVITY PRE-CHECK! 
            // If he gets heavily tilted AND has experienced a lift, we definitively know a human grabbed him.
            // He can NO LONGER accidentally trigger this by just being lowered down!
            if (abs(currentAngles.pitch) > 25.0f || abs(currentAngles.roll) > 25.0f) {
                if (hasExperiencedLift) {
                    isHandling = true;
                }
            }

            // THE FIX 7.2: Unlatching the human hand.
            // If he is perfectly flat AND the smoothed energy is rock solid (< 4.0), the human let go.
            bool isUpright = (abs(currentAngles.pitch) < 5.0f && abs(currentAngles.roll) < 5.0f);
            if (isUpright && smoothedTotalEnergy < 4.0f) {
                isHandling = false;
                hasExperiencedLift = false; // Reset the gravity lock so he must be lifted anew next time!
            }

            if (activeMode != &compassMode) {
                // ENTRY LOGIC: Differentiating active play vs holding steady.
                if (isHandling) {
                    // THE FIX 6: 3-Second Compass Entry Delay!
                    // He is in human hands. But is he being wildly rotated, or held steady on a book?
                    // A steady book hold generates moderate energy (< 150). Active rotation easily spikes > 200.
                    if (totalRawEnergy < 150.0f) {
                        if (!pickupTimerActive) {
                            pickupTimerActive = true;
                            pickupStartTime = millis();
                        } else if (millis() - pickupStartTime >= 3000) {
                            // Held steady for 3 seconds! Safe to lock compass!
                            activeMode = &compassMode;
                            pickupTimerActive = false;
                            settlingTimerActive = false; // Reset the floor exit timer
                        }
                    } else {
                        // Too much movement! (Being actively rotated/played with). Break the timer!
                        pickupTimerActive = false;
                    }
                } else {
                    pickupTimerActive = false;
                }
            }
            // HYSTERESIS EXIT: Differentiating the Hand vs The Floor
            else if (activeMode == &compassMode) {
                if (!isHandling) {
                    // THE FIX 7.3: The "Floor Unlatch"
                    // We changed from 10.0f to 4.0f. A steady human hand easily averages 8-10. 
                    // A solid wooden table averages 1.5 to 2.5. 4.0 is the perfect razor's edge!
                    bool isPerfectlyStill = (smoothedTotalEnergy < 4.0f);
                    
                    if (isPerfectlyStill) {
                        if (!settlingTimerActive) {
                            // We just leveled out and stopped vibrating. Start the 3-second countdown!
                            settlingTimerActive = true;
                            settlingStartTime = millis();
                            activeMode = &compassMode; 
                        } else if (millis() - settlingStartTime <= 3000) {
                            // Timer is ticking, hold the mode...
                            activeMode = &compassMode; 
                        } else {
                            // 3 seconds have passed upright AND perfectly still. We are safe on the ground.
                            activeMode = &normalMode; 
                            settlingTimerActive = false; // Reset for next time
                        }
                    } else {
                        // Wait, it is upright, but vibrating (noise spike). Just hold the lock!
                        settlingTimerActive = false; 
                        activeMode = &compassMode;   
                    }
                } else {
                    // Either we are tilted, OR we are upright but actively vibrating (human hand). 
                    // Reset the timer and hold the lock!
                    settlingTimerActive = false; 
                    activeMode = &compassMode;   
                }
            }
            // PRIORITY 4: Default Behavior
            else {
                activeMode = &normalMode; 
            }
        } // End of Priority 3 & 4 Block

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
    }
}

/*
// ==========================================
// CORE 1: THE MAIN CONTROL LOOP
// ==========================================
void ControlLoopTask(void *pvParameters) { 
    IRobotMode* previousMode = nullptr; // To track mode changes for onEnter/onExit

    // Static variables for our Hysteresis Timers (Preserved between loop ticks)
    unsigned long settlingStartTime = 0;
    bool settlingTimerActive = false;

    // === THE SOFTWARE METRONOME SETUP ===
    // 10ms = 100Hz (The professional standard for flight controllers)
    const TickType_t xFrequency = pdMS_TO_TICKS(10);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        // Lock the loop to exactly 100Hz. 
        // This ensures dt in the Madgwick filter is always perfectly accurate!
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

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
        
        // Calculate the Distance Delta
        float distanceDelta = 0.0f;
        if (distance != lastDistance && lastDistance > 0.0f) {
            distanceDelta = distance - lastDistance;
        }
        lastDistance = distance;
             
        auto getShortestAngleDelta = [](float current, float previous) {
            float delta = current - previous;
            if (delta > 180.0f) delta -= 360.0f;
            else if (delta < -180.0f) delta += 360.0f;
            
            // THE FIX 1: Do not take the absolute value here! 
            // Return the true signed (+ or -) movement so vibrations cancel out.
            return delta;
        };

        // --- NEW KINETIC ENERGY "DIZZY BAR" LOGIC ---
        // 1. Calculate the Raw Kinetic Energy (Absolute Speed)
        // We take the abs() here because we want to measure the total energy spent shaking him!
        float rawYawEnergy = abs(getShortestAngleDelta(currentAngles.yaw, lastAngles.yaw)) / 0.01f;
        float rawPitchEnergy = abs(getShortestAngleDelta(currentAngles.pitch, lastAngles.pitch)) / 0.01f;
        float rawRollEnergy = abs(getShortestAngleDelta(currentAngles.roll, lastAngles.roll)) / 0.01f;
        
        // 2. THE KINETIC ENERGY ACCUMULATOR
        // THE FIX 2: Increased charge rate from 0.05 to 0.20 so human shaking actually fills the bar!
        static float dizzyBarYaw = 0.0f;
        static float dizzyBarPitch = 0.0f;
        static float dizzyBarRoll = 0.0f;

        dizzyBarYaw = (0.20f * rawYawEnergy) + (0.80f * dizzyBarYaw);
        dizzyBarPitch = (0.20f * rawPitchEnergy) + (0.80f * dizzyBarPitch);
        dizzyBarRoll = (0.20f * rawRollEnergy) + (0.80f * dizzyBarRoll);

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
        // 💥 NEW LOCK 1C: Hysteresis for Distance Game (Prevent fluttering at the trigger edge)
        // If the hand backs up to 26cm (trigger is 25cm), don't instantly drop the game.
        // Wait until the hand is fully pulled away (> 40cm) before returning to normal!
        else if (activeMode == &distanceMode && distance > 0 && distance < (PersonalityConfig::OBSTACLE_TRIGGER_DISTANCE_CM + 15.0f)) {
            activeMode = &distanceMode; 
        }

        // PRIORITY 2: THE DIZZY TRAP (Re-promoted!)
        // THE FIX 3: Lowered the threshold from 400 to 250 so it responds better to human muscle speed.
        // If the Dizzy Bar hits 250, trigger dizzy mode and override Compass Lock!
        else if ((dizzyBarYaw > 250.0f || dizzyBarPitch > 250.0f || dizzyBarRoll > 250.0f) && !isDizzy) {
            isDizzy = true;
            dizzyStartTime = millis();
            activeMode = &dizzyMode;
            
            // Instantly drain the energy bars so he doesn't re-trigger immediately after waking up!
            dizzyBarYaw = 0.0f; dizzyBarPitch = 0.0f; dizzyBarRoll = 0.0f;
        }
        
        // Handle Dizzy Timer
        // The Untrapped Timer
        // Let the timer run independently so it doesn't freeze if picked up.
        else if (isDizzy) {
            if (millis() - dizzyStartTime > 10000) {
                isDizzy = false; 
                // Only return to normal if he isn't currently being held by a human
                if (activeMode == &dizzyMode) {
                    activeMode = &normalMode; 
                }
            } else {
                activeMode = &dizzyMode; // Force him back to dizzy if Priority 1 temporarily interrupted it!
            }
        }

        // PRIORITY 3: PHYSICAL HANDLING (Compass Lock)
        // This only runs if he IS NOT dizzy. 
        // THE FIX 4: Based on raw telemetry, resting flat is ~0.1 Pitch and ~-3.5 Roll.
        // HYSTERESIS ENTER: Require a heavy 25-degree tilt to initially engage.
        else if (abs(currentAngles.pitch) > 25.0f || abs(currentAngles.roll) > 25.0f) {
            activeMode = &compassMode;
            settlingTimerActive = false; // Reset the timer, we are actively tilted!
        }
        // HYSTERESIS EXIT: Differentiating the Hand vs The Floor
        else if (activeMode == &compassMode) {
            // THE FIX 5: "Upright" means safely within 15 degrees of zero on both axes, 
            // easily accounting for his natural -3.5 degree roll and slightly uneven floors.
            bool isUpright = (abs(currentAngles.pitch) < 15.0f && abs(currentAngles.roll) < 15.0f);
            
            // THE FIX 6: The Floor is dead quiet. A human hand vibrates. 
            // A change of just 0.15 degrees per 10ms frame equals 15 deg/sec of energy.
            // If the raw energy is below 15 on all axes, he is perfectly still on a hard surface.
            bool isPerfectlyStill = (rawYawEnergy < 15.0f && rawPitchEnergy < 15.0f && rawRollEnergy < 15.0f);
            
            if (isUpright && isPerfectlyStill) {
                if (!settlingTimerActive) {
                    // We just leveled out and stopped vibrating. Start the 3-second countdown!
                    settlingTimerActive = true;
                    settlingStartTime = millis();
                    activeMode = &compassMode; 
                } else if (millis() - settlingStartTime <= 3000) {
                    // Timer is ticking, hold the mode...
                    activeMode = &compassMode; 
                } else {
                    // 3 seconds have passed upright AND perfectly still. We are safe on the ground.
                    activeMode = &normalMode; 
                    settlingTimerActive = false; // Reset for next time
                }
            } else {
                // Either we are tilted, OR we are upright but vibrating (human hand). 
                // Reset the timer and hold the lock!
                settlingTimerActive = false; 
                activeMode = &compassMode;   
            }
        }

        // PRIORITY 4: Default Behavior
        else {
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
    }
} */

// ==========================================
// THE COMMAND LINE INTERFACE (CLI) ENGINE
// ==========================================
void executeCLICommand(String cmd) {
    CLI::CommandCode code = CLI::parseCommand(cmd);

    switch (code) {
        case CLI::CommandCode::CALIB_GYRO:
            imu->calibrateGyro();
            break;
        case CLI::CommandCode::CALIB_ACCEL:
            imu->calibrateAccel();
            break;
        case CLI::CommandCode::CALIB_MAG:
            imu->calibrateMag();
            break;
        case CLI::CommandCode::CALIB_SONAR:
            logger.println("[CLI] Sonar calibration not yet implemented.");
            break;
        case CLI::CommandCode::UNKNOWN:
        default:
            CLI::printHelpMenu(logger);
            break;
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
  static String cliBuffer = ""; // Memory to hold the word as you type it

  for (;;) {
    logger.handleClient();

    // === THE CLI LISTENER ===
    while (Serial.available()) {
        char incomingChar = Serial.read();

        // If the user presses 'Enter'
        if (incomingChar == '\n' || incomingChar == '\r') {
            if (cliBuffer.length() > 0) {
                executeCLICommand(cliBuffer); 
                cliBuffer = ""; // Clear memory for next command
            }
        } else {
            cliBuffer += incomingChar; 
        }
    }

    // 1. Ping the HC-SR04
    global_frontDistanceCM = frontSonar.getDistanceCM();

    // 2. TRANSMIT TELEMETRY
    if (global_imuAlive) {
        logger.printf("Sonar: %.1f cm | Y: %5.1f | P: %5.1f | R: %5.1f\n", 
                      global_frontDistanceCM, global_yaw, global_pitch, global_roll);
    } else {
        logger.printf("Sonar: %.1f cm | IMU: DEAD (ZOMBIE STATE)\n", 
                      global_frontDistanceCM);
    }

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