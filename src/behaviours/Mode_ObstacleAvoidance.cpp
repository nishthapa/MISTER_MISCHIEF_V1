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

void Mode_ObstacleAvoidance::onEnter(const volatile GlobalDataBank& robotData) {
    logger.println("Mister Mischief has encountered an obstacle! Initiating Radial Sweep...");
    pingCount = 0;
    lastPingTime = millis();
    entryHeading = robotData.physics.imuAngles.yaw; // Snapshot from memory!
    changeState(BACKING_UP);
}

void Mode_ObstacleAvoidance::update(const RobotMood& currentMood, const volatile GlobalDataBank& robotData) {
    unsigned long elapsed = millis() - stateStartTime;
    
    //float speed = ObstacleConfig::BASE_SPEED * currentMood.speedMultiplier;
    
    float speed = SysConfig.CRUISING_SPEED * currentMood.speedMultiplier;

    switch (currentState) {
        case BACKING_UP:
            kinematics->rawDrive(-speed, -speed);
            if (elapsed > SysConfig.OBSTACLE_BACKUP_DURATION_MS) changeState(RADAR_SWEEP);
            break;

        case RADAR_SWEEP:
            kinematics->rawDrive(-speed, speed);
            
            if (millis() - lastPingTime > 50 && pingCount < MAX_PINGS) {
                pointCloud[pingCount].heading = robotData.physics.imuAngles.yaw;
                pointCloud[pingCount].distance = robotData.sensors.distanceCM;
                pingCount++;
                lastPingTime = millis();
            }

            // 2. IMU-DRIVEN SWEEP: Stop sweeping exactly when we have turned 160 degrees. 
            // (Timeout added just in case the robot gets physically stuck)
            if (abs(getShortestAngle(robotData.physics.imuAngles.yaw, entryHeading)) >= SysConfig.OBSTACLE_SWEEP_ANGLE_DEG || elapsed > SysConfig.OBSTACLE_SWEEP_TIMEOUT_MS) {
                changeState(CALCULATING);
            }
            break;

        case CALCULATING:
            kinematics->stop();
            {
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