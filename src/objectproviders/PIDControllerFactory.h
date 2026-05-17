#pragma once
#include "core/PIDController.h"
#include "config/ConfigurationManager.h" 

class PIDControllerFactory {
public:
    static PIDController createHeadingHoldPID() {
        return PIDController(Config.PID_HEADING_P, Config.PID_HEADING_I, Config.PID_HEADING_D, 
                             Config.PID_HEADING_LIM, Config.PID_HEADING_ILIM, Config.PID_HEADING_DEAD);
    }

    static PIDController createCompassLockPID() {
        return PIDController(Config.PID_COMPASS_P, Config.PID_COMPASS_I, Config.PID_COMPASS_D, 
                             Config.PID_COMPASS_LIM, Config.PID_COMPASS_ILIM, Config.PID_COMPASS_DEAD);
    }

    static PIDController createDistanceHoldPID() {
        return PIDController(Config.PID_DIST_P, Config.PID_DIST_I, Config.PID_DIST_D, 
                             Config.PID_DIST_LIM, Config.PID_DIST_ILIM, Config.PID_DIST_DEAD);
    }

    static PIDController createObstacleAvoidanceNewPathScanSweepPID() {
        return PIDController(Config.PID_OBSTACLE_P, Config.PID_OBSTACLE_I, Config.PID_OBSTACLE_D, 
                             Config.PID_OBSTACLE_LIM, Config.PID_OBSTACLE_ILIM, Config.PID_OBSTACLE_DEAD);
    }
};