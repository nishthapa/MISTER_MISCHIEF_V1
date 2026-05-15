#pragma once

#include "hal/interfaces/I_IMU.h"
#include "hal/hardware/distancesensor/HCSR04_Sonar.h"
#include "behaviours/IRobotMode.h"
#include "behaviours/RobotMood.h"

// Forward declarations of your modes so the Brain knows they exist
class Mode_ObstacleAvoidance;
class Mode_NormalDriving;
class Mode_CompassLock;
class Mode_MaintainDistance;
class Mode_Dizzy;
class Mode_DeepSleep;

class BehaviourEngine {
private:
    // The Hardware
    I_IMU* imu;
    I_DistanceSensor* sonar;

    // The Modes
    Mode_ObstacleAvoidance* obstacleMode;
    Mode_NormalDriving* normalMode;
    Mode_CompassLock* compassMode;
    Mode_MaintainDistance* distanceMode;
    Mode_Dizzy* dizzyMode;
    Mode_DeepSleep* sleepMode;

    // The active states
    IRobotMode* activeMode;
    IRobotMode* previousMode;
    RobotMood activeMood;

    // The Physics & Memory State
    float frustrationLevel;
    bool isGroggyPhase;
    unsigned long coldBootTime;
    
    bool isDizzy;
    unsigned long dizzyStartTime;
    
    bool isHandling;
    bool hasExperiencedLift;
    unsigned long settlingStartTime;
    bool settlingTimerActive;
    unsigned long pickupStartTime;
    bool pickupTimerActive;

    // --- Drop & Settle Latches ---
    bool isLowering;
    bool hasLanded;

    float lastDistance;
    FusedAngles lastAngles;

    // Hidden Kinetic Energy Accumulators
    float dizzyBarYaw, dizzyBarPitch, dizzyBarRoll;
    float smoothedTotalEnergy;

public:
    BehaviourEngine(I_IMU* i, I_DistanceSensor* s, 
                    Mode_ObstacleAvoidance* obs, Mode_NormalDriving* norm, 
                    Mode_CompassLock* comp, Mode_MaintainDistance* dist, 
                    Mode_Dizzy* diz, Mode_DeepSleep* sleep);

    void init(bool isColdBoot);
    
    // The main 100Hz tick
    void update();

    // The elegant getters for telemetry!
    const char* getActiveModeName() const;
    const char* getActiveMoodName() const;
};