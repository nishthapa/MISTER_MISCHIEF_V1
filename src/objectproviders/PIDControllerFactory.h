#pragma once
#include "core/PIDController.h"
#include "config/ConfigurationManager.h" 

class PIDControllerFactory {
public:
    static PIDController createPointTurnPID() {
        return PIDController(SysConfig.PID_POINT_P, SysConfig.PID_POINT_I, SysConfig.PID_POINT_D, 
                             SysConfig.PID_POINT_LIM, SysConfig.PID_POINT_ILIM, SysConfig.PID_POINT_DEAD);
    }

    static PIDController createArcTurnPID() {
        return PIDController(SysConfig.PID_ARC_P, SysConfig.PID_ARC_I, SysConfig.PID_ARC_D, 
                             SysConfig.PID_ARC_LIM, SysConfig.PID_ARC_ILIM, SysConfig.PID_ARC_DEAD);
    }

    static PIDController createDistanceHoldPID() {
        return PIDController(SysConfig.PID_DIST_P, SysConfig.PID_DIST_I, SysConfig.PID_DIST_D, 
                             SysConfig.PID_DIST_LIM, SysConfig.PID_DIST_ILIM, SysConfig.PID_DIST_DEAD);
    }
};