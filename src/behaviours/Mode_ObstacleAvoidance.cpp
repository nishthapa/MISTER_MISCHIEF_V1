#include "behaviours/Mode_ObstacleAvoidance.h"
#include <Arduino.h>
#include "utils/RemoteLogger.h" // For logging the radar sweep results and chosen escape path
// #include "config/ObstacleAvoidanceConfig.h" // Safe to DELETE since we're now pulling these values from the NVS-backed ConfigurationManager
#include "config/ConfigurationManager.h"

Mode_ObstacleAvoidance::Mode_ObstacleAvoidance(KinematicsEngine* k, I_DistanceSensor* s, I_IMU* i) {
    kinematics = k; sonar = s; imu = i;
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

void Mode_ObstacleAvoidance::onEnter() {
    logger.println("Mister Mischief has encountered an obstacle! Initiating Radial Sweep...");
    pingCount = 0;
    lastPingTime = millis();
    entryHeading = imu->getAngles().yaw; // Take a snapshot of the direction we were travelling
    changeState(BACKING_UP);
}

void Mode_ObstacleAvoidance::update(const RobotMood& currentMood) {
    unsigned long elapsed = millis() - stateStartTime;
    
    //float speed = ObstacleConfig::BASE_SPEED * currentMood.speedMultiplier;
    
    float speed = Config.CRUISING_SPEED * currentMood.speedMultiplier;

    switch (currentState) {
        case BACKING_UP:
            kinematics->rawDrive(-speed, -speed);
            if (elapsed > Config.OBSTACLE_BACKUP_DURATION_MS) changeState(RADAR_SWEEP);
            break;

        case RADAR_SWEEP:
            kinematics->rawDrive(-speed, speed);
            
            if (millis() - lastPingTime > 50 && pingCount < MAX_PINGS) {
                pointCloud[pingCount].heading = imu->getAngles().yaw;
                pointCloud[pingCount].distance = sonar->getDistanceCM();
                pingCount++;
                lastPingTime = millis();
            }

            // 2. IMU-DRIVEN SWEEP: Stop sweeping exactly when we have turned 160 degrees. 
            // (Timeout added just in case the robot gets physically stuck)
            if (abs(getShortestAngle(imu->getAngles().yaw, entryHeading)) >= Config.OBSTACLE_SWEEP_ANGLE_DEG || 
                elapsed > Config.OBSTACLE_SWEEP_TIMEOUT_MS) {
                changeState(CALCULATING);
            }
            break;

        case CALCULATING:
            kinematics->stop();
            {
                float bestScore = -1.0f;
                bestEscapeHeading = entryHeading; // Fallback

                for (int i = 0; i < pingCount; i++) {
                    // Filter out sonar misfires
                    if (pointCloud[i].distance <= 0.0f) continue;

                    // 3. THE PENALTY MATH (Ping-Pong Prevention)
                    // Calculate how far off-center this angle is (0 to 180 degrees)
                    float deviation = abs(getShortestAngle(pointCloud[i].heading, entryHeading));
                    
                    // The Aggressive Base Penalty (0° = 1.0, 90° = 0.5, 180° = 0.0)
                    float basePenalty = 1.0f - (deviation / 180.0f); 

                    // Square it to brutally punish backward angles!
                    // (e.g., a 150° turn gets its distance multiplied by a tiny 0.027)
                    float aggressivePenalty = basePenalty * basePenalty; 

                    float score = pointCloud[i].distance * aggressivePenalty;

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
                float error = getShortestAngle(bestEscapeHeading, imu->getAngles().yaw);
                
                // Zero math here! The kinematics engine handles alignment natively now
                kinematics->navigateToHeading(bestEscapeHeading, imu->getAngles().yaw, 0.0f, currentMood.pidAggression);

                // ADDED: The infinite loop timeout fix we discussed earlier
                if (abs(error) < Config.OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG || 
                    elapsed > Config.OBSTACLE_ALIGN_TIMEOUT_MS) {
                    changeState(ESCAPE);
                }
            }
            break;

        case ESCAPE:
            kinematics->rawDrive(speed, speed);
            if (elapsed > Config.OBSTACLE_ESCAPE_DURATION_MS) changeState(FINISHED); 
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