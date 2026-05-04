#pragma once

#include "core/PIDController.h"
#include "config/pidtuning/PIDTune_HeadingHold.h" // For Heading hold during normal driving
#include "config/pidtuning/PIDTune_BalancePlatform.h" // For Heading Hold on a rotating platform

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

    // A static "vending machine" function to dispense a pre-tuned Balance Platform PID
    static PIDController createBalancePlatformPID() {
        return PIDController(
            TuningBalancePlatform::KP, 
            TuningBalancePlatform::KI, 
            TuningBalancePlatform::KD, 
            TuningBalancePlatform::MAX_INTEGRAL,
            TuningBalancePlatform::MAX_OUTPUT,
            TuningBalancePlatform::D_FILTER_CUTOFF_HZ
        );
    }
};