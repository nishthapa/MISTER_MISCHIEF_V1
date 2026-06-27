#pragma once

#include "hal/interfaces/I_IMU.h"
#include "hal/hardware/distancesensor/HCSR04_Sonar.h"
#include "behaviours/IRobotMode.h"
#include "behaviours/RobotMood.h"
#include "core/RobotState.h"
#include "core/GlobalDataBus.h"
#include "core/EventLatchHandler.h"

class Mode_ObstacleAvoidance;
class Mode_NormalDriving;
class Mode_CompassLock;
class Mode_MaintainDistance;
class Mode_Dizzy;
class Mode_DeepSleep;
class Mode_Teleop; 
class Mode_Diagnostics;
class Mode_AutoTune;
class Mode_BrainDead;

class BehaviourEngine {
private:

    Mode_ObstacleAvoidance* obstacleMode;
    Mode_NormalDriving* normalMode;
    Mode_CompassLock* compassMode;
    Mode_MaintainDistance* distanceMode;
    Mode_Dizzy* dizzyMode;
    Mode_DeepSleep* sleepMode;
    Mode_Teleop* teleopMode; 
    Mode_Diagnostics* diagnosticMode;
    Mode_AutoTune* autotuneMode;
    Mode_BrainDead* brainDeadMode;
    
    IRobotMode* activeMode;
    IRobotMode* previousMode;
    RobotMood activeMood;

    EventLatchHandler latchHandler; // <--- Add it here
    
    bool isGroggyPhase;
    unsigned long coldBootTime;

    SystemMode mapModeToEnum(IRobotMode* mode);
    IRobotMode* getModeFromEnum(SystemMode modeEnum);

    // --- NEW: The AI Virtual Sensor ---
    void runAIPerception();

public:
    BehaviourEngine(Mode_ObstacleAvoidance* obs, Mode_NormalDriving* norm, 
                    Mode_CompassLock* comp, Mode_MaintainDistance* dist, 
                    Mode_Dizzy* diz, Mode_DeepSleep* sleep, Mode_Teleop* teleop,
                    Mode_Diagnostics* diag, Mode_AutoTune* autot,
                    Mode_BrainDead* deadMode);

    void init(bool isColdBoot);
    void update(const GlobalDataBank& robotData);
    void changeMode(IRobotMode* newMode);
    
    const char* getActiveModeName() const;
    const char* getActiveMoodName() const;
};