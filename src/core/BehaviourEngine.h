#pragma once

#include "hal/interfaces/I_IMU.h"
#include "hal/hardware/distancesensor/HCSR04_Sonar.h"
#include "behaviours/IRobotMode.h"
#include "behaviours/RobotMood.h"
#include "core/RobotState.h"
#include "core/EventLatchHandler.h" // <--- The new Handler!
#include "core/GlobalSensorState.h" // <--- The new Global Sensor State!

class Mode_ObstacleAvoidance;
class Mode_NormalDriving;
class Mode_CompassLock;
class Mode_MaintainDistance;
class Mode_Dizzy;
class Mode_DeepSleep;
class Mode_Teleop; // <--- Forward declaration of the new Teleop mode

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
    Mode_Teleop* teleopMode; // <--- ADD THIS
    IRobotMode* activeMode;
    IRobotMode* previousMode;
    RobotMood activeMood;

    EventLatchHandler latchHandler; // <--- Replaces 20 earlier variables!
    
    bool isGroggyPhase;
    unsigned long coldBootTime;
    float lastDistance;
    FusedAngles lastAngles;

    PerceptionData gatherPerception(const volatile GlobalSensorState& sensorState);
    IRobotMode* determineNextMode(const SemanticEvents& events);
    SystemMode mapModeToEnum(IRobotMode* mode);

public:
    BehaviourEngine(Mode_ObstacleAvoidance* obs, Mode_NormalDriving* norm, 
                    Mode_CompassLock* comp, Mode_MaintainDistance* dist, 
                    Mode_Dizzy* diz, Mode_DeepSleep* sleep, Mode_Teleop* teleop);

    void init(bool isColdBoot);
    void update(const volatile GlobalSensorState& sensorState);
    void changeMode(IRobotMode* newMode);
    
    const char* getActiveModeName() const;
    const char* getActiveMoodName() const;
};