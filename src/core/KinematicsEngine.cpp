#include "core/KinematicsEngine.h"
#include "config/SystemConfig.h"
#include "config/ConfigurationManager.h"

// 1. Constructor Updated
KinematicsEngine::KinematicsEngine(PIDController* pointPID, PIDController* arcPID) {
    pointTurnPID = pointPID;
    arcTurnPID = arcPID;
}

void KinematicsEngine::reloadPIDTunings() {
    // 1. Unified sync of the Point Turn PID
    pointTurnPID->setTunings(
        SysConfig.PID_POINT_P, SysConfig.PID_POINT_I, SysConfig.PID_POINT_D,
        SysConfig.PID_POINT_ILIM, SysConfig.PID_POINT_LIM
    );

    // 2. Unified sync of the Arc Turn PID
    arcTurnPID->setTunings(
        SysConfig.PID_ARC_P, SysConfig.PID_ARC_I, SysConfig.PID_ARC_D,
        SysConfig.PID_ARC_ILIM, SysConfig.PID_ARC_LIM
    );
}

float KinematicsEngine::getShortestAngle(float target, float current) {
    float delta = target - current;
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    return delta;
}

// 2. Navigation Updated
void KinematicsEngine::navigateToHeading(float targetYaw, float currentYaw, float baseSpeed, float moodAggression) {
    float error = getShortestAngle(targetYaw, currentYaw);
    float dt = SystemConfig::MAIN_LOOP_TICK_RATE_MS / 1000.0f;
    float steeringCorrection = 0.0f;

    // Kinematic Profiler: Choose PID profile based on state
    if (baseSpeed == 0.0f) {
        // Stationary: Apply high proportional authority to break static scrub friction
        
        // ADDED THE MINUS SIGN BACK SINCE YAW DIRECTION NOW FOLLOWS ESTABLISHED CONVENTIONS!
        steeringCorrection = pointTurnPID->compute(0.0f, -error, dt); 
    } else {
        // Rolling: Apply damped heading-hold adjustments for low dynamic friction
        
        // ADDED THE MINUS SIGN BACK SINCE YAW DIRECTION NOW FOLLOWS ESTABLISHED CONVENTIONS!
        steeringCorrection = arcTurnPID->compute(0.0f, -error, dt);   
    }
    
    steeringCorrection *= moodAggression;

    // Kinematic Mixer
    float leftPWM = baseSpeed + steeringCorrection;
    float rightPWM = baseSpeed - steeringCorrection;
    
    //motorDriver->drive(leftPWM, rightPWM);

    // 3. Update telemetry immediately after command
    //reportTelemetry(leftPWM, rightPWM);

    // SAVE THE ANSWERS, DO NOT TOUCH HARDWARE
    outTargetHeading = targetYaw;
    outHeadingError = error;
    outLeftPWM = static_cast<int16_t>(leftPWM);
    outRightPWM = static_cast<int16_t>(rightPWM);

    updateMovementFlags();


    // if (outLeftPWM || outRightPWM) {
    //     isDriving = true;
    // } else {
    //     isDriving = false;
    // }
}

// 3. Raw Drive Updated
void KinematicsEngine::rawDrive(float leftSpeed, float rightSpeed) {
    outTargetHeading = 0.0f;
    outHeadingError = 0.0f;
    outLeftPWM = static_cast<int16_t>(leftSpeed);
    outRightPWM = static_cast<int16_t>(rightSpeed);

    updateMovementFlags();
    
    // if (outLeftPWM || outRightPWM) {
    //     isDriving = true;
    // } else {
    //     isDriving = false;
    // }
}

// 4. Stop Updated
void KinematicsEngine::stop() {
    outTargetHeading = 0.0f;
    outHeadingError = 0.0f;
    outLeftPWM = 0;
    outRightPWM = 0;
    //isDriving = false;
    updateMovementFlags();
}

// FLAGS CAN CO-EXIST! BETTER FOR DETERMINISM!
void KinematicsEngine::updateMovementFlags() {
    // Reset all flags to false initially
    isDrivingStraightForward        = false;
    isDrivingStraightReverse        = false;
    isPointTurningLeft              = false;
    isPointTurningRight             = false;
    isArcTurningForwardLeft         = false;
    isArcTurningForwardRight        = false;
    isArcTurningReverseLeft         = false;
    isArcTurningReverseRight        = false;
    isOneSidedTurningForwardLeft    = false;
    isOneSidedTurningForwardRight   = false;
    isOneSidedTurningReverseLeft    = false;
    isOneSidedTurningReverseRight   = false;

    // 1. Master driving status evaluation
    isDriving = (outLeftPWM != 0 || outRightPWM != 0);
    if (!isDriving) {
        return;
    }

    const int16_t TOLERANCE = SystemConfig::DRIVING_DIRECTION_DETERMINATION_TOLERANCE;
    int16_t pwmDifference   = outLeftPWM - outRightPWM;
    int16_t pwmSum          = outLeftPWM + outRightPWM;

    // 2. MACRO STATE: STRAIGHT MOTION (Within drift tolerance, same direction)
    if (abs(pwmDifference) <= TOLERANCE) {
        if (outLeftPWM > 0 && outRightPWM > 0)      isDrivingStraightForward = true;
        else if (outLeftPWM < 0 && outRightPWM < 0) isDrivingStraightReverse = true;
    }
    
    // 3. MACRO STATE: POINT TURNS (Within balance tolerance, opposite directions)
    else if (abs(pwmSum) <= TOLERANCE) {
        if (outLeftPWM < 0 && outRightPWM > 0)      isPointTurningLeft = true;     
        else if (outLeftPWM > 0 && outRightPWM < 0) isPointTurningRight = true; 
    }
    
    // 4. MACRO STATE: ARC TURNS (Unequal, non-zero tracks, moving in same general direction)
    // Note: We check if they share signs, or if one is zero but the other drives
    else if ((outLeftPWM >= 0 && outRightPWM >= 0) || (outLeftPWM <= 0 && outRightPWM <= 0)) {
        // Evaluate forward vs reverse arcs based on the dominant/faster moving track
        bool isForward = (abs(outLeftPWM) > abs(outRightPWM)) ? (outLeftPWM > 0) : (outRightPWM > 0);
        
        if (isForward) {
            if (outLeftPWM < outRightPWM)           isArcTurningForwardLeft  = true;   
            else if (outLeftPWM > outRightPWM)      isArcTurningForwardRight = true; 
        } else { 
            if (outLeftPWM > outRightPWM)           isArcTurningReverseLeft  = true;   
            else if (outLeftPWM < outRightPWM)      isArcTurningReverseRight = true; 
        }
    }

    // 5. MICRO STATE EXTRACTION: ONE-SIDED PIVOT CONDITIONS
    // This runs completely independently to flag if a track is stalled or dragging
    if (abs(outLeftPWM) <= TOLERANCE) {
        if (outRightPWM > TOLERANCE)        isOneSidedTurningForwardLeft  = true; 
        else if (outRightPWM < -TOLERANCE)  isOneSidedTurningReverseLeft  = true; 
    }
    else if (abs(outRightPWM) <= TOLERANCE) {
        if (outLeftPWM > TOLERANCE)         isOneSidedTurningForwardRight = true; 
        else if (outLeftPWM < -TOLERANCE)   isOneSidedTurningReverseRight = true; 
    }
}

// STRICTLY MUTUALLY EXCLUSIVE! DIFFICULT TO CODE STATE MACHINE! DISCARDED!
// void KinematicsEngine::updateMovementFlags() {
//     // Reset all flags to false initially
//     isDrivingStraightForward        = false;
//     isDrivingStraightReverse        = false;
//     isPointTurningLeft              = false;
//     isPointTurningRight             = false;
//     isArcTurningForwardLeft         = false;
//     isArcTurningForwardRight        = false;
//     isArcTurningReverseLeft         = false;
//     isArcTurningReverseRight        = false;
//     isOneSidedTurningForwardLeft    = false;
//     isOneSidedTurningForwardRight   = false;
//     isOneSidedTurningReverseLeft    = false;
//     isOneSidedTurningReverseRight   = false;

//     // 1. Master driving status evaluation
//     isDriving = (outLeftPWM != 0 || outRightPWM != 0);

//     // Stop condition: if both are zero, exit early
//     if (!isDriving) {
//         return;
//     }

//     // 2. Define the tolerance window (allows +/- 3 PWM ticks of drift)
//     const int16_t TOLERANCE = SystemConfig::DRIVING_DIRECTION_DETERMINATION_TOLERANCE;
//     int16_t pwmDifference   = outLeftPWM - outRightPWM;
//     int16_t pwmSum          = outLeftPWM + outRightPWM;

//     // 3. PRIORITY 1: ONE-SIDED TURNS (One track is mechanically considered stopped)
//     if (abs(outLeftPWM) <= TOLERANCE) {
//         // Left track is stopped. Right track is moving.
//         if (outRightPWM > TOLERANCE)        isOneSidedTurningForwardLeft  = true; 
//         else if (outRightPWM < -TOLERANCE)  isOneSidedTurningReverseLeft  = true; 
//     }
//     else if (abs(outRightPWM) <= TOLERANCE) {
//         // Right track is stopped. Left track is moving.
//         if (outLeftPWM > TOLERANCE)         isOneSidedTurningForwardRight = true; 
//         else if (outLeftPWM < -TOLERANCE)   isOneSidedTurningReverseRight = true; 
//     }

//     // 4. PRIORITY 2: STRAIGHT MOTION (Mismatches within tolerance, moving in same direction)
//     else if (abs(pwmDifference) <= TOLERANCE) {
//         if (outLeftPWM > 0 && outRightPWM > 0)      isDrivingStraightForward = true;
//         else if (outLeftPWM < 0 && outRightPWM < 0) isDrivingStraightReverse = true;
//     }
    
//     // 5. PRIORITY 3: POINT TURNS (Mismatches within tolerance, moving in opposite directions)
//     else if (abs(pwmSum) <= TOLERANCE) {
//         if (outLeftPWM < 0 && outRightPWM > 0)      isPointTurningLeft = true;     
//         else if (outLeftPWM > 0 && outRightPWM < 0) isPointTurningRight = true; 
//     }
    
//     // 6. PRIORITY 4: ARC TURNS (Same direction, but variance breaches all tolerance limits)
//     else if ((outLeftPWM > 0 && outRightPWM > 0) || (outLeftPWM < 0 && outRightPWM < 0)) {
//         bool isForward = (outLeftPWM > 0);
        
//         if (isForward) {
//             if (outLeftPWM < outRightPWM)           isArcTurningForwardLeft  = true;   
//             else if (outLeftPWM > outRightPWM)      isArcTurningForwardRight = true; 
//         } else { 
//             if (outLeftPWM > outRightPWM)           isArcTurningReverseLeft  = true;   
//             else if (outLeftPWM < outRightPWM)      isArcTurningReverseRight = true; 
//         }
//     }
// }


// TOO-STRICT!!! NO DEADBAND! DISCARDED
// void KinematicsEngine:: updateMovementFlags() {
//     // Reset all flags to false initially
//     isDrivingStraightForward  = false;
//     isDrivingStraightReverse  = false;
//     isPointTurningLeft        = false;
//     isPointTurningRight       = false;
//     isArcTurningForwardLeft   = false;
//     isArcTurningForwardRight  = false;
//     isArcTurningReverseLeft   = false;
//     isArcTurningReverseRight  = false;

//     // Stop condition: if both are zero, exit early
//     if (outLeftPWM == 0 && outRightPWM == 0) {
//         return;
//     }

//     // 1. STRAIGHT MOTION (Equal speeds, same direction)
//     if (outLeftPWM == outRightPWM) {
//         if (outLeftPWM > 0) isDrivingStraightForward = true;
//         else if (outLeftPWM < 0) isDrivingStraightReverse = true;
//     }
    
//     // 2. POINT TURNS (Equal speeds, opposite directions)
//     else if (outLeftPWM == -outRightPWM) {
//         if (outLeftPWM < 0) isPointTurningLeft = true;     // Left back, right forward
//         else if (outLeftPWM > 0) isPointTurningRight = true; // Left forward, right back
//     }
    
//     // 3. ARC TURNS (Same direction, different non-zero speeds)
//     else if ((outLeftPWM > 0 && outRightPWM > 0) || (outLeftPWM < 0 && outRightPWM < 0)) {
//         bool isForward = (outLeftPWM > 0);
        
//         if (isForward) {
//             if (outLeftPWM < outRightPWM) isArcTurningForwardLeft = true;   // Right pushes harder
//             else if (outLeftPWM > outRightPWM) isArcTurningForwardRight = true; // Left pushes harder
//         } else { // Reverse
//             // Note: lower value means higher reverse speed magnitude (e.g., -200 vs -100)
//             if (outLeftPWM > outRightPWM) isArcTurningReverseLeft = true;   // Right reverses harder
//             else if (outLeftPWM < outRightPWM) isArcTurningReverseRight = true; // Left reverses harder
//         }
//     }

//     // 4. If either track has a non-zero speed, the robot is driving
//     isDriving = (outLeftPWM != 0 || outRightPWM != 0);
// }