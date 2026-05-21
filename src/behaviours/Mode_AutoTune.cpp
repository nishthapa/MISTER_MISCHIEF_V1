#include "behaviours/Mode_AutoTune.h"
#include "config/ConfigurationManager.h"
#include "utils/RemoteLogger.h"
#include "core/BehaviourEngine.h" // For turninng brain (BRAIN_ACTIVE) on after autotune finishes
#include "behaviours/Mode_NormalDriving.h" // For switching back to Normal Driving after autotune finishes
#include <math.h>

// We need access to the live PID to update it instantly after the tune
extern PIDController pointTurnPID; 
extern PIDController arcTurnPID;

extern BehaviourEngine brain;
extern Mode_NormalDriving normalMode;

Mode_AutoTune::Mode_AutoTune(I_IMU* i, KinematicsEngine* k) {
    imu = i; kinematics = k;
    tuneState = STATE_FINISHED;
}

float Mode_AutoTune::getShortestAngle(float target, float current) {
    float delta = target - current;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    return delta;
}

void Mode_AutoTune::onEnter() {
    logger.println("\n[AUTOTUNE] Sequence Initiated. DO NOT TOUCH ROBOT.");
    kinematics->stop();
    tuneState = STATE_COUNTDOWN;

    countdownSeconds = Config.AUTOTUNE_START_DELAY_MS / 1000;
    lastCountdownTime = millis();

    if (countdownSeconds > 0) {
        logger.print("Autotune initiating in "); // Start the inline string
    }

}

void Mode_AutoTune::update(const RobotMood& currentMood) {
    float currentYaw = imu->getAngles().yaw;

    // THE DYNAMIC LOADING INDICATOR (........)
    // If we are actively calibrating or wobbling, print a dot every 1 second!
    if ((tuneState == STATE_STATIC_CAL || tuneState == STATE_RELAY_WOBBLE) && (millis() - lastDotTime >= 1000)) {
        logger.print("."); 
        lastDotTime = millis();
    }
    
    switch(tuneState) {
        // ==========================================
        // PHASE 0: THE ASYNC COUNTDOWN (5..4..3..2..1)
        // ==========================================
        case STATE_COUNTDOWN:
            if (millis() - lastCountdownTime >= 1000) {
                lastCountdownTime = millis();
                
                if (countdownSeconds > 0) {
                    // If it's the last second, don't print the dots, just the number and a newline
                    if (countdownSeconds > 1) {
                        logger.printf("%d...", countdownSeconds);
                    } else {
                        logger.printf("%d\n", countdownSeconds);
                    }
                    countdownSeconds--;
                } else {
                    // Start the line that the loading dots will attach to
                    logger.print("Autotuning"); 
                    lastDotTime = millis();
                    
                    tuneState = STATE_STATIC_CAL;
                    stateStartTime = millis();
                }
            }
            break;

        // ==========================================
        // PHASE 1: GYRO THERMAL CALIBRATION
        // ==========================================
        case STATE_STATIC_CAL:
            // Give the robot 2 seconds to settle physically before calibrating
            if (millis() - stateStartTime > 2000) {
                logger.println("[AUTOTUNE] Zeroing Gyroscope...");
                
                // Assuming your IMU has a calibrate/zero function
                imu->calibrateGyro();
                // imu->calibrateAccel(); // Only do this on an absolutely level surface! YOU HAVE BEEN WARNED!
                // Config.CALIBRATED_TEMP_C = imu->getTemperature(); 
                // ConfigSys.save();
                // logger.printf("[AUTOTUNE] Thermal Fingerprint saved at: %.1f C\n", Config.CALIBRATED_TEMP_C); 
                
                startYaw = currentYaw; // Snapshot the exact starting heading
                
                // Reset variables for Phase 2
                wobbleCount = 0;
                maxAmplitude = 0.0f;
                totalPeriodMs = 0;
                turningRight = true;
                lastCrossTime = millis();
                
                tuneState = STATE_RELAY_WOBBLE;
                logger.println("[AUTOTUNE] Commencing Relay Wobble Test...");
            }
            break;

        // ==========================================
        // PHASE 2: ZIEGLER-NICHOLS WOBBLE
        // ==========================================
        case STATE_RELAY_WOBBLE:
        {
            // === THE HARDWARE TIMEOUT LATCH ===
            if (millis() - lastCrossTime > Config.AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS) {
                kinematics->stop();
                logger.println("\n\n[AUTOTUNE] Failed. Please check if hardware is properly connected/working and try again.");
                tuneState = STATE_FINISHED;
                break; 
            }
            float error = getShortestAngle(startYaw, currentYaw);
            
            // Track the maximum overshoot (Amplitude)
            if (abs(error) > maxAmplitude) {
                maxAmplitude = abs(error);
            }

            // The Relay Logic (Bang-Bang Control)
            if (turningRight) {
                kinematics->rawDrive(RELAY_PWM, -RELAY_PWM);
                if (error < -WOBBLE_TOLERANCE) { // Crossed the left boundary!
                    turningRight = false;
                    wobbleCount++;
                    totalPeriodMs += (millis() - lastCrossTime);
                    lastCrossTime = millis();
                }
            } else {
                kinematics->rawDrive(-RELAY_PWM, RELAY_PWM);
                if (error > WOBBLE_TOLERANCE) { // Crossed the right boundary!
                    turningRight = true;
                    wobbleCount++;
                    totalPeriodMs += (millis() - lastCrossTime);
                    lastCrossTime = millis();
                }
            }

            // Check if we have collected enough data
            if (wobbleCount >= REQUIRED_WOBBLES) {
                kinematics->stop();
                calculateAndSavePID(); // Do the math!
                
                // Prepare for Phase 3
                startYaw = currentYaw; 
                tuneState = STATE_VICTORY_SPIN;
                logger.println("[AUTOTUNE] Tuning complete! Executing Victory Spin.");
            }
            break;
        }

        // ==========================================
        // PHASE 3: VERIFICATION & MAG CALIBRATION
        // ==========================================
        case STATE_VICTORY_SPIN:
        {
            // Target is +360 from where we finished the wobble
            float target = startYaw + 360.0f;
            float error = getShortestAngle(target, currentYaw);
            
            // Use the NEWLY TUNED PID to make this turn perfectly smooth!
            kinematics->navigateToHeading(target, currentYaw, 0.0f, currentMood.pidAggression);

            // [FUTURE PHASE 2]: Add Magnetometer Min/Max reading logic here!
            
            // Check if turn is complete (within 2 degrees)
            if (abs(error) < 2.0f) {
                kinematics->stop();
                logger.println("[AUTOTUNE] Sequence Success! Returning to normal ops.");
                tuneState = STATE_FINISHED;
            }
            break;
        }

        case STATE_FINISHED:
            // 1. Re-enable the autonomous brain!
            Config.BRAIN_ACTIVE = true;
            
            // 2. Hand control back to the normal driving state
            brain.changeMode(&normalMode);
            
            // 3. Reset the internal state so Autotune can be run again later
            tuneState = STATE_COUNTDOWN; 
            break;
    }
}

void Mode_AutoTune::calculateAndSavePID() {
    // 1. Calculate Average Period (in seconds)
    // A full period is TWO wobbles (left then right), so we multiply the average half-period by 2
    float avgHalfPeriodMs = (float)totalPeriodMs / (float)REQUIRED_WOBBLES;
    float Tu = (avgHalfPeriodMs * 2.0f) / 1000.0f; // Ultimate Period in seconds
    
    // 2. Calculate Ultimate Gain (Ku)
    // The standard relay formula: Ku = (4 * drive) / (pi * amplitude)
    float Ku = (4.0f * RELAY_PWM) / (PI * maxAmplitude);

    // 3. Apply Ziegler-Nichols rules for a classic PID controller
    float newKp = 0.6f * Ku;
    float newKi = 1.2f * Ku / Tu;
    float newKd = 0.075f * Ku * Tu;

    logger.printf("\n\n[AUTOTUNE] Ku: %.2f | Tu: %.2fs | Max Amp: %.2f deg\n", Ku, Tu, maxAmplitude);
    logger.printf("[AUTOTUNE] New POINT PID -> P:%.2f I:%.2f D:%.2f\n", newKp, newKi, newKd);

    // 4. Arc Turn (Rolling - Low Friction)
    // Dynamic friction is significantly lower. We scale the authority down.
    float arcKp = newKp * 0.65f; 
    float arcKi = newKi * 0.50f;
    float arcKd = newKd * 0.80f; // Keep D relatively high to prevent cruising wobbles

    // 5. Save to global config and NVS
    Config.PID_POINT_P = newKp;
    Config.PID_POINT_I = newKi;
    Config.PID_POINT_D = newKd;
    Config.PID_ARC_P = arcKp;
    Config.PID_ARC_I = arcKi;
    Config.PID_ARC_D = arcKd;
    ConfigSys.save();

    // 5. Update the live object instantly so Phase 3 can use it!
    pointTurnPID.setTunings(newKp, newKi, newKd, Config.PID_POINT_ILIM, Config.PID_POINT_LIM);
    arcTurnPID.setTunings(arcKp, arcKi, arcKd, Config.PID_ARC_ILIM, Config.PID_ARC_LIM);
}

void Mode_AutoTune::onExit() {
    kinematics->stop();
}