#include "behaviours/Mode_Diagnostics.h"
#include "core/GlobalDataBus.h"
#include "utils/RemoteLogger.h"
#include <Arduino.h>

extern RemoteLogger logger;

Mode_Diagnostics::Mode_Diagnostics(KinematicsEngine* k) {
    kinematics = k;
}

void Mode_Diagnostics::onEnter(const GlobalDataBank& robotData) {
    kinematics->stop();
    testState = 0;
    stateStartTime = millis();
    logger.println("[DIAGNOSTICS] Motor Test Initializing in 2 seconds...");
}

void Mode_Diagnostics::update(const RobotMood& currentMood, const GlobalDataBank& robotData) {
    unsigned long currentTime = millis();

    // 1. Read the memory bus to see if the user typed an answer
    HardwareCommandBus cmdSnapshot;
    portENTER_CRITICAL(&hardwareCmdLock);
    cmdSnapshot = HardwareCommands;
    portEXIT_CRITICAL(&hardwareCmdLock);

    // 2. Non-Blocking State Machine
    switch(testState) {
        case 0: // Wait 2 seconds before Left Test
            if (currentTime - stateStartTime >= 2000) {
                logger.println("\n[DIAGNOSTICS] Spinning LEFT Channel Positive...");
                testState = 1;
                stateStartTime = currentTime;
            }
            break;

        case 1: // Run LEFT motor for 5 seconds
            kinematics->rawDrive(50.0f, 0.0f); 
            
            if (currentTime - stateStartTime >= 5000) {
                kinematics->stop(); 
                logger.println("\nWhat physically happened on the robot?");
                logger.println("  1 = Left track moved FORWARD");
                logger.println("  2 = Left track moved REVERSE");
                logger.println("  3 = Right track moved FORWARD");
                logger.println("  4 = Right track moved REVERSE");
                logger.println("  5 = Nothing moved at all");
                logger.print("Enter your answer (1-5): ");
                testState = 2; 
            }
            break;

        case 2: // Wait for LEFT answer
            kinematics->stop(); 
            if (cmdSnapshot.diagnosticAnswer >= 1 && cmdSnapshot.diagnosticAnswer <= 5) {
                leftMotorTestAns = cmdSnapshot.diagnosticAnswer;
                
                // Clear the answer from the bus so it doesn't instantly trigger the next phase!
                portENTER_CRITICAL(&hardwareCmdLock);
                HardwareCommands.diagnosticAnswer = 0;
                portEXIT_CRITICAL(&hardwareCmdLock);

                logger.println("\nGot it. Now testing RIGHT Motor Channel (Positive PWM) in 2 seconds...");
                testState = 3;
                stateStartTime = currentTime;
            }
            break;

        case 3: // Wait 2 seconds before Right Test
            kinematics->stop();
            if (currentTime - stateStartTime >= 2000) {
                logger.println("[DIAGNOSTICS] Spinning RIGHT Channel Positive...");
                testState = 4;
                stateStartTime = currentTime;
            }
            break;

        case 4: // Run RIGHT motor for 5 seconds
            kinematics->rawDrive(0.0f, 50.0f); 
            
            if (currentTime - stateStartTime >= 5000) {
                kinematics->stop(); 
                logger.println("\nWhat physically happened on the robot?");
                logger.println("  1 = Left track moved FORWARD");
                logger.println("  2 = Left track moved REVERSE");
                logger.println("  3 = Right track moved FORWARD");
                logger.println("  4 = Right track moved REVERSE");
                logger.println("  5 = Nothing moved at all");
                logger.print("Enter your answer (1-5): ");
                testState = 5; 
            }
            break;

        case 5: // Wait for RIGHT answer & DIAGNOSE
            kinematics->stop();
            if (cmdSnapshot.diagnosticAnswer >= 1 && cmdSnapshot.diagnosticAnswer <= 5) {
                rightMotorTestAns = cmdSnapshot.diagnosticAnswer;
                
                logger.println("\n\n=== DIAGNOSIS RESULTS ===");
                
                // --- YOUR EXACT DIAGNOSIS LOGIC ---
                if (leftMotorTestAns == 5 && rightMotorTestAns == 5) {
                    logger.println("[DEAD] Neither motor moved.");
                    logger.println("Check ENA/ENB wiring, main battery power, and ensure the XY-160D logic pins are connected.");
                } 
                else if (leftMotorTestAns == 1 && rightMotorTestAns == 3) {
                    logger.println("[PERFECT] Your motors are wired flawlessly! No changes needed.");
                } 
                else {
                    logger.println("[ISSUE DETECTED] Incorrect wiring mapped.");
                    logger.println("\n--- HOW TO FIX IT ---");
                    logger.println("Open src/config/PinConfig.h and update these pins:\n");
                    
                    logger.print("LEFT CHANNEL is currently driving: ");
                    if (leftMotorTestAns == 1) logger.println("Left Forward (Correct)");
                    else if (leftMotorTestAns == 2) logger.println("Left Reverse. \n  -> FIX: Swap PIN_MOTOR_LEFT_FWD and PIN_MOTOR_LEFT_REV");
                    else if (leftMotorTestAns == 3) logger.println("Right Forward. \n  -> FIX: This channel is swapped with the right side!");
                    else if (leftMotorTestAns == 4) logger.println("Right Reverse. \n  -> FIX: Swapped sides AND reversed polarity.");
                    
                    logger.print("RIGHT CHANNEL is currently driving: ");
                    if (rightMotorTestAns == 3) logger.println("Right Forward (Correct)");
                    else if (rightMotorTestAns == 4) logger.println("Right Reverse. \n  -> FIX: Swap PIN_MOTOR_RIGHT_FWD and PIN_MOTOR_RIGHT_REV");
                    else if (rightMotorTestAns == 1) logger.println("Left Forward. \n  -> FIX: This channel is swapped with the left side!");
                    else if (rightMotorTestAns == 2) logger.println("Left Reverse. \n  -> FIX: Swapped sides AND reversed polarity.");
                    
                    if ((leftMotorTestAns == 3 || leftMotorTestAns == 4) && (rightMotorTestAns == 1 || rightMotorTestAns == 2)) {
                        logger.println("\n[MASTER FIX] You completely crossed the Left and Right sides.");
                        logger.println("  -> The easiest fix is to physically swap the Left and Right motor wires where they screw into the XY160D.");
                    }
                }
                logger.println("=== WIZARD COMPLETE ===\n");

                // Clear the flags to allow BehaviourEngine to resume Normal Driving
                portENTER_CRITICAL(&hardwareCmdLock);
                HardwareCommands.requestMotorTest = false;
                HardwareCommands.diagnosticAnswer = 0;
                portEXIT_CRITICAL(&hardwareCmdLock);
                
                testState = 6;
            }
            break;

        case 6: // Idle until the Engine context-switches us out
            kinematics->stop();
            break;
    }
}

void Mode_Diagnostics::onExit() {
    kinematics->stop();
    logger.println("[DIAGNOSTICS] Mode Exited.");
}