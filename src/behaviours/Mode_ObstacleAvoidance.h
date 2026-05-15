#pragma once

#include "behaviours/IRobotMode.h"
#include "hal/XY160D_MotorDriver.h"
#include "hal/HCSR04_Sonar.h"
#include "hal/I_IMU.h"
#include "core/PIDController.h"

// A struct to hold our "Point Cloud" data
struct RadarPing {
    float heading;
    float distance;
};

class Mode_ObstacleAvoidance : public IRobotMode {
private:
    XY160D_MotorDriver* motors;
    HCSR04_Sonar* sonar;
    I_IMU* imu;
    PIDController* alignPID;

    // The streamlined Cockroach dance
    enum SequenceState { BACKING_UP, RADAR_SWEEP, CALCULATING, ALIGNING, ESCAPE, FINISHED };
    SequenceState currentState;
    unsigned long stateStartTime;
    unsigned long lastPingTime;

    // Radar Memory (Store up to 30 pings per sweep)
    static const int MAX_PINGS = 30;
    RadarPing pointCloud[MAX_PINGS];
    int pingCount = 0;

    float entryHeading; // Remember where we were going!
    float bestEscapeHeading;

    void changeState(SequenceState newState);
    float getShortestAngle(float target, float current);

public:
    // Dependency Injection upgraded
    Mode_ObstacleAvoidance(XY160D_MotorDriver* m, HCSR04_Sonar* s, I_IMU* i, PIDController* p);
    
    void onEnter() override;
    void update(const RobotMood& currentMood) override;
    const char* getName() const override { return "MODE_OBSTACLE_AVOIDANCE"; }
    void onExit() override;
    bool isSequenceComplete();
};