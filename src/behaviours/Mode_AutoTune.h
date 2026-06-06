#pragma once
#include "behaviours/IRobotMode.h"
#include "core/KinematicsEngine.h"

enum AutoTuneState {
    STATE_COUNTDOWN, // <-- NEW 1st State i.e. counting down until autotune starts as defined in SysConfig.AUTOTUNE_START_DELAY_MS
    STATE_STATIC_CAL,
    STATE_RELAY_WOBBLE,
    STATE_VICTORY_SPIN,
    STATE_FINISHED
};

class Mode_AutoTune : public IRobotMode {
private:
    KinematicsEngine* kinematics; // NO IMU!

    AutoTuneState tuneState;

    unsigned long lastCountdownTime;
    int countdownSeconds;
    unsigned long lastDotTime; // For the "Autotuning......" dots

    unsigned long stateStartTime;

    // --- Ziegler-Nichols Tracking Variables ---
    float startYaw;
    int wobbleCount;
    float maxAmplitude;
    unsigned long lastCrossTime;
    unsigned long totalPeriodMs;
    bool turningRight;

    // Tuning Constants
    const float RELAY_PWM = 120.0f;     // The raw motor power used to test friction
    const float WOBBLE_TOLERANCE = 5.0f; // Degrees to drift before reversing
    const int REQUIRED_WOBBLES = 10;    // Number of half-cycles to average

    float getShortestAngle(float target, float current);
    void calculateAndSavePID();

public:
    Mode_AutoTune(KinematicsEngine* k);

    void onEnter(const volatile GlobalSensorState& sensorState) override;
    void update(const RobotMood& currentMood, const volatile GlobalSensorState& sensorState) override;
    const char* getName() const override { return "MODE_AUTOTUNE"; }
    void onExit() override;
};