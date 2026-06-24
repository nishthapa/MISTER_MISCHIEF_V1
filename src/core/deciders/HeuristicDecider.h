#pragma once

#include "core/interfaces/IBehaviourDecider.h"
#include "core/EventLatchHandler.h"
#include "config/SystemConfig.h"
#include "behaviours/Mode_ObstacleAvoidance.h" 

class HeuristicDecider : public IBehaviourDecider {
private:
    EventLatchHandler latchHandler;
    Mode_ObstacleAvoidance* obstacleMode;
    
    float lastDistance;
    bool hasBaro;
    FusedAngles lastAngles;

    PerceptionData gatherPerception(const GlobalDataBank& robotData);
    SystemMode determineNextMode(SystemMode activeMode, const SemanticEvents& events, const GlobalDataBank& robotData);

public:
    HeuristicDecider(Mode_ObstacleAvoidance* obs);
    
    void evaluate(const GlobalDataBank& dataBus, SystemMode currentMode, SystemMode& outRequestedMode, RobotMood& outMood) override;
    
    // Expose the latch handler so BehaviourEngine can still write telemetry data
    const EventLatchHandler& getLatchHandler() const { return latchHandler; }
};