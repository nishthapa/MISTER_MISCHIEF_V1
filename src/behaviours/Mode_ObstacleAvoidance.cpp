#include "behaviours/Mode_ObstacleAvoidance.h"
#include <Arduino.h>
#include "utils/RemoteLogger.h" // For logging the radar sweep results and chosen escape path

Mode_ObstacleAvoidance::Mode_ObstacleAvoidance(XY160D_MotorDriver* m, HCSR04_Sonar* s, MPU6050_IMU* i, PIDController* p) {
    motors = m; sonar = s; imu = i; alignPID = p;
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
    Serial.println("Obstacle! Initiating Radial Sweep...");
    logger.println("Obstacle! Initiating Radial Sweep...");
    pingCount = 0;
    lastPingTime = millis();
    entryHeading = imu->getAngles().yaw; // Take a snapshot of the direction we were travelling
    changeState(BACKING_UP);
}

void Mode_ObstacleAvoidance::update(const RobotMood& currentMood) {
    unsigned long elapsed = millis() - stateStartTime;
    float speed = 40.0f * currentMood.speedMultiplier; 

    switch (currentState) {
        case BACKING_UP:
            motors->drive(-speed, -speed);
            if (elapsed > 500) changeState(RADAR_SWEEP);
            break;

        case RADAR_SWEEP:
            motors->drive(-speed, speed); 
            
            if (millis() - lastPingTime > 50 && pingCount < MAX_PINGS) {
                pointCloud[pingCount].heading = imu->getAngles().yaw;
                pointCloud[pingCount].distance = sonar->getDistanceCM();
                pingCount++;
                lastPingTime = millis();
            }

            // 2. IMU-DRIVEN SWEEP: Stop sweeping exactly when we have turned 160 degrees. 
            // (Timeout added just in case the robot gets physically stuck)
            if (abs(getShortestAngle(imu->getAngles().yaw, entryHeading)) >= 160.0f || elapsed > 3000) {
                changeState(CALCULATING);
            }
            break;

        case CALCULATING:
            motors->stop();
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
                Serial.printf("Best path found at %.1f degrees. Aligning...\n", bestEscapeHeading);
                logger.printf("Best path found at %.1f degrees. Aligning...\n", bestEscapeHeading);
                changeState(ALIGNING);
            }
            break;

        case ALIGNING:
            {
                float currentYaw = imu->getAngles().yaw;
                float error = getShortestAngle(bestEscapeHeading, currentYaw);
                
                float correction = alignPID->compute(0.0f, -error, 0.01f);
                float finalCorrection = correction * currentMood.pidAggression;
                
                motors->drive(finalCorrection, -finalCorrection);

                if (abs(error) < 5.0f) changeState(ESCAPE);
            }
            break;

        case ESCAPE:
            motors->drive(speed, speed);
            if (elapsed > 1000) changeState(FINISHED); 
            break;

        case FINISHED:
            motors->stop(); 
            break;
    }
}

void Mode_ObstacleAvoidance::onExit() {
    motors->stop();
}

bool Mode_ObstacleAvoidance::isSequenceComplete() {
    return (currentState == FINISHED);
}