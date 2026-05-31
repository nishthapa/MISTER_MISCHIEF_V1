#pragma once

#include "hal/interfaces/I_IMU.h"
#include "hal/hardware/distancesensor/HCSR04_Sonar.h"
#include "behaviours/IRobotMode.h"
#include "behaviours/RobotMood.h"
#include "core/RobotState.h" // <--- The new global variable!

class Mode_ObstacleAvoidance;
class Mode_NormalDriving;
class Mode_CompassLock;
class Mode_MaintainDistance;
class Mode_Dizzy;
class Mode_DeepSleep;

// --- THE PERCEPTION STRUCT ---
// A clean package of all the math for the current frame
struct PerceptionData {
    float currentDistance;
    float distanceDelta;
    float currentGForce;
    float totalRawEnergy;
    float rawYawEnergy;
    float rawPitchEnergy;
    float rawRollEnergy;
    bool isUpright;
};

class BehaviourEngine {
private:
    I_IMU* imu;
    I_DistanceSensor* sonar;

    Mode_ObstacleAvoidance* obstacleMode;
    Mode_NormalDriving* normalMode;
    Mode_CompassLock* compassMode;
    Mode_MaintainDistance* distanceMode;
    Mode_Dizzy* dizzyMode;
    Mode_DeepSleep* sleepMode;

    IRobotMode* activeMode;
    IRobotMode* previousMode;
    RobotMood activeMood;

    // --- State Memory ---
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
    bool isLowering;
    bool hasLanded;
    bool isHandVanishing;
    unsigned long vanishingStartTime;
    bool isHandTeasing;            
    unsigned long teaseStartTime;  
    float ambientBackgroundDistance; 
    
    float lastDistance;
    FusedAngles lastAngles;
    float dizzyBarYaw, dizzyBarPitch, dizzyBarRoll;
    float smoothedTotalEnergy;

    // --- THE NEW ARCHITECTURE METHODS ---
    PerceptionData gatherPerception();
    void updateMoodTracker(const PerceptionData& p);
    IRobotMode* determineNextMode(const PerceptionData& p);
    SystemMode mapModeToEnum(IRobotMode* mode);

public:
    BehaviourEngine(I_IMU* i, I_DistanceSensor* s, 
                    Mode_ObstacleAvoidance* obs, Mode_NormalDriving* norm, 
                    Mode_CompassLock* comp, Mode_MaintainDistance* dist, 
                    Mode_Dizzy* diz, Mode_DeepSleep* sleep);

    void init(bool isColdBoot);
    void update();
    void changeMode(IRobotMode* newMode);
    
    const char* getActiveModeName() const;
    const char* getActiveMoodName() const;
};