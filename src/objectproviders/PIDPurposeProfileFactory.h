#pragma once

#include "core/PIDController.h"
#include "config/pidtuning/PIDTune_HeadingHold.h" // For Heading hold during normal driving
#include "config/pidtuning/PIDTune_CompassLock.h" // For Heading Hold on a rotating platform and act like a compass
#include "config/pidtuning/PIDTune_DistanceHold.h" // For holding distance from an object (like hand)

class PIDPurposeProfileFactory {
public:
    // A static "vending machine" function to dispense a pre-tuned Heading PID
    static PIDController createHeadingHoldPID() {
        return PIDController(
            TuningHeading::KP, 
            TuningHeading::KI, 
            TuningHeading::KD, 
            TuningHeading::MAX_INTEGRAL,
            TuningHeading::MAX_OUTPUT,
            TuningHeading::D_FILTER_CUTOFF_HZ
        );
    }

    // A static "vending machine" function to dispense a pre-tuned Compass Lock PID
    static PIDController createCompassLockPID() {
        return PIDController(
            TuningCompassLock::KP, 
            TuningCompassLock::KI, 
            TuningCompassLock::KD, 
            TuningCompassLock::MAX_INTEGRAL,
            TuningCompassLock::MAX_OUTPUT,
            TuningCompassLock::D_FILTER_CUTOFF_HZ
        );
    }

    // A static "vending machine" function to dispense a pre-tuned Distance Hold PID
    static PIDController createDistanceHoldPID() {
        return PIDController(
            TuningDistanceHold::KP, 
            TuningDistanceHold::KI, 
            TuningDistanceHold::KD, 
            TuningDistanceHold::MAX_INTEGRAL,
            TuningDistanceHold::MAX_OUTPUT,
            TuningDistanceHold::D_FILTER_CUTOFF_HZ
        );
    }
};