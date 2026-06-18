#include "behaviours/Mode_ObstacleAvoidance.h"
#include <Arduino.h>
#include "utils/RemoteLogger.h" // For logging the radar sweep results and chosen escape path
// #include "config/ObstacleAvoidanceConfig.h" // Safe to DELETE since we're now pulling these values from the NVS-backed ConfigurationManager
#include "config/ConfigurationManager.h"

Mode_ObstacleAvoidance::Mode_ObstacleAvoidance(KinematicsEngine* k) {
    kinematics = k;
    currentState = FINISHED;
}

void Mode_ObstacleAvoidance::changeState(SequenceState newState) {
    currentState = newState;
    stateStartTime = millis();
}

float Mode_ObstacleAvoidance::getShortestAngle(float target, float current) {
    float delta = target - current;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    return delta;
}

void Mode_ObstacleAvoidance::onEnter(const GlobalDataBank& robotData) {
    logger.println("Mister Mischief has encountered an obstacle! Initiating Radial Sweep...");
    pingCount = 0;
    lastPingTime = millis();
    entryHeading = robotData.physics.imuAngles.yaw; // Snapshot from memory!
    changeState(BACKING_UP);
}

void Mode_ObstacleAvoidance::update(const RobotMood& currentMood, const GlobalDataBank& robotData) {
    unsigned long elapsed = millis() - stateStartTime;
    float speed = SysConfig.CRUISING_SPEED * currentMood.speedMultiplier;

    switch (currentState) {
        case BACKING_UP:
            kinematics->rawDrive(-speed, -speed);
            if (elapsed > SysConfig.OBSTACLE_BACKUP_DURATION_MS) changeState(SWEEP_LEFT);
            break;

        case SWEEP_LEFT:
        case SWEEP_RIGHT:
            {
                // 1. WHICH WAY ARE WE SWEEPING?
                float sweepPWM = 65.0f; // Slow, deliberate scan speed // To-do: Move to NVS config storage
                // Expanded to 95 degrees to ensure we see parallel escape routes!
                float targetOffset = (currentState == SWEEP_LEFT) ? -95.0f : 95.0f; // To-do: Move to SystemConfig
                
                if (currentState == SWEEP_LEFT) {
                    kinematics->rawDrive(-sweepPWM, sweepPWM); // Spin Left
                } else {
                    kinematics->rawDrive(sweepPWM, -sweepPWM); // Spin Right
                }

                // 2. THE ALIASING LATCH
                // Only log data when Task_Sensor actually pushes a fresh ping to memory
                static float lastLoggedDistance = -1.0f;
                float currentDistance = robotData.sensors.distanceCM;

                if (currentDistance != lastLoggedDistance && pingCount < MAX_PINGS) {
                    pointCloud[pingCount].heading = robotData.physics.imuAngles.yaw;
                    pointCloud[pingCount].distance = currentDistance;
                    lastLoggedDistance = currentDistance;
                    pingCount++;
                }

                // 3. DYNAMIC STALL DETECTION (The "Hand Grab" failsafe)
                static float lastWatchdogYaw = robotData.physics.imuAngles.yaw;
                static unsigned long lastWatchdogTime = millis();
                
                if (millis() - lastWatchdogTime > 500) { // Check every half-second
                    float yawMoved = abs(getShortestAngle(robotData.physics.imuAngles.yaw, lastWatchdogYaw));
                    
                    if (yawMoved < 5.0f) { // If it moved less than 5 degrees in 500ms, it is physically stuck!
                        kinematics->stop();
                        logger.println("[RADAR] PHYSICAL STALL DETECTED! Aborting sweep to protect motors.");
                        changeState(CALCULATING);
                        break;
                    }
                    lastWatchdogYaw = robotData.physics.imuAngles.yaw;
                    lastWatchdogTime = millis();
                }

                // 4. TRANSITION LOGIC
                float degreesFromEntry = getShortestAngle(entryHeading, robotData.physics.imuAngles.yaw);
                
                if (currentState == SWEEP_LEFT && degreesFromEntry <= targetOffset) {
                    // Reached the left limit! Now sweep right.
                    kinematics->stop();
                    delay(100); // Give the robot a tiny physical pause (looks very robotic)
                    changeState(SWEEP_RIGHT);
                    lastWatchdogTime = millis(); // Reset watchdog for the new direction
                } 
                else if (currentState == SWEEP_RIGHT && degreesFromEntry >= targetOffset) {
                    // Reached the right limit! We are done sweeping.
                    kinematics->stop();
                    logger.println("[RADAR] Head scan complete. Analyzing point cloud...");
                    changeState(CALCULATING);
                }
                
                // Bumped timeout to 8 seconds to allow the massive 190-degree total pan
                if (elapsed > 8000) changeState(CALCULATING);
            }
            break;

        case CALCULATING:
            kinematics->stop();
            {
                // Failsafe in case stall detection triggered instantly
                if (pingCount == 0) {
                    logger.println("[RADAR] Zero data collected! Executing blind 180 fallback.");
                    bestEscapeHeading = entryHeading + 180.0f;
                    if (bestEscapeHeading > 360.0f) bestEscapeHeading -= 360.0f;
                    changeState(ALIGNING);
                    break;
                }

                float bestScore = -1.0f;
                bestEscapeHeading = entryHeading; // Fallback

                for (int i = 0; i < pingCount; i++) {
                    if (pointCloud[i].distance <= 0.0f) continue;
                    
                    float deviation = abs(getShortestAngle(pointCloud[i].heading, entryHeading));
                    float scoreMultiplier = 1.0f; // Default to trusting the raw distance

                    // ==========================================
                    // THE PING-PONG PREVENTION
                    // ==========================================
                    
                    // If the angle is pointing mostly backwards (> 135 degrees), 
                    // it is the path we came from. Slash its score by 80%!
                    if (deviation > 135.0f) {
                        scoreMultiplier = 0.2f;
                    }
                    // If the angle is pointing straight back into the wall we just hit,
                    // apply a mild 50% penalty just to be safe.
                    else if (deviation < 30.0f) {
                        scoreMultiplier = 0.5f;
                    }
                    // Angles between 30 and 135 (left and right flanks) keep a 1.0 multiplier!
                    float score = pointCloud[i].distance * scoreMultiplier;

                    if (score > bestScore) {
                        bestScore = score;
                        bestEscapeHeading = pointCloud[i].heading;
                    }
                }
                logger.printf("Best path found at %.1f degrees. Aligning...\n", bestEscapeHeading);
                changeState(ALIGNING);
            }
            break;

        case ALIGNING:
            {
                float error = getShortestAngle(bestEscapeHeading, robotData.physics.imuAngles.yaw);
                
                // Zero math here! The kinematics engine handles alignment natively now
                kinematics->navigateToHeading(bestEscapeHeading, robotData.physics.imuAngles.yaw, 0.0f, currentMood.pidAggression);
                
                // ADDED: The infinite loop timeout fix we discussed earlier
                if (abs(error) < SysConfig.OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG || 
                    elapsed > SysConfig.OBSTACLE_ALIGN_TIMEOUT_MS) {
                    changeState(ESCAPE);
                }
            }
            break;

        case ESCAPE:
            kinematics->rawDrive(speed, speed);
            if (elapsed > SysConfig.OBSTACLE_ESCAPE_DURATION_MS) changeState(FINISHED);
            break;

        case FINISHED:
            kinematics->stop();
            break;
    }
}

void Mode_ObstacleAvoidance::onExit() {
    kinematics->stop();
}

bool Mode_ObstacleAvoidance::isSequenceComplete() {
    return (currentState == FINISHED);
}