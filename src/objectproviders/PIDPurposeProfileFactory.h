#pragma once

#include "core/PIDController.h"
#include "config/pidtuning/PIDTune_HeadingHold.h"
// #include "config/pidtuning/PIDTune_BalancePlatform.h" // Add this later!

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

    // Later, you just add another button to the vending machine:
    /*
    static PIDController createPartyTrickPID() {
        return PIDController(TuningTrick::KP, ...);
    }
    */
};