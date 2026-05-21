#pragma once
#include "core/PIDController.h"
#include "config/ConfigurationManager.h" 

class PIDControllerFactory {
public:
    static PIDController createPointTurnPID() {
        return PIDController(Config.PID_POINT_P, Config.PID_POINT_I, Config.PID_POINT_D, 
                             Config.PID_POINT_LIM, Config.PID_POINT_ILIM, Config.PID_POINT_DEAD);
    }

    static PIDController createArcTurnPID() {
        return PIDController(Config.PID_ARC_P, Config.PID_ARC_I, Config.PID_ARC_D, 
                             Config.PID_ARC_LIM, Config.PID_ARC_ILIM, Config.PID_ARC_DEAD);
    }

    static PIDController createDistanceHoldPID() {
        return PIDController(Config.PID_DIST_P, Config.PID_DIST_I, Config.PID_DIST_D, 
                             Config.PID_DIST_LIM, Config.PID_DIST_ILIM, Config.PID_DIST_DEAD);
    }
};