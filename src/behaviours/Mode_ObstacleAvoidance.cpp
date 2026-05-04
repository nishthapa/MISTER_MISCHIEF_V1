#include "behaviours/Mode_ObstacleAvoidance.h"
#include <Arduino.h>

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
                    // How far backward is this angle? (0 = straight ahead, 180 = directly behind)
                    float deviation = abs(getShortestAngle(pointCloud[i].heading, entryHeading));
                    
                    // We severely penalize going backward. 
                    // Formula: $Score = Distance \times (1 - \frac{Deviation}{360})$
                    // A path 180 degrees behind him gets its perceived distance cut in half!
                    float penalty = 1.0f - (deviation / 360.0f); 
                    float score = pointCloud[i].distance * penalty;

                    if (score > bestScore) {
                        bestScore = score;
                        bestEscapeHeading = pointCloud[i].heading;
                    }
                }
                Serial.printf("Best path found at %.1f degrees. Aligning...\n", bestEscapeHeading);
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