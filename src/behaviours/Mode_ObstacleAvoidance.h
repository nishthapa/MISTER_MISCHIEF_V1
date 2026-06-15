#pragma once

#include "behaviours/IRobotMode.h"
#include "core/KinematicsEngine.h" //For the new kinematics engine


// A struct to hold our "Point Cloud" data
struct RadarPing {
    float heading;
    float distance;
};

class Mode_ObstacleAvoidance : public IRobotMode {
private:
    KinematicsEngine* kinematics; // NO SONAR OR IMU POINTERS! Handled by the RobotData now

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
    Mode_ObstacleAvoidance(KinematicsEngine* k);
    
    void onEnter(const GlobalDataBank& robotData) override;
    void update(const RobotMood& currentMood, const GlobalDataBank& robotData) override;
    const char* getName() const override { return "MODE_OBSTACLE_AVOIDANCE"; }
    void onExit() override;
    
    bool isSequenceComplete();
};