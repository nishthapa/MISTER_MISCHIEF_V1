#pragma once

#include <Arduino.h>
#include "hal/interfaces/I_MotorDriver.h" // <-- Include the contract

class XY160D_MotorDriver : public I_MotorDriver {
private:
    // Variable names fixed to match your .cpp file!
    int enaPin, leftFwdPin, leftRevPin, rightFwdPin, rightRevPin, enbPin;

public:
    XY160D_MotorDriver(int ena, int lf, int lr, int rf, int rr, int enb);

    // Notice we removed the accidental semicolon before override!
    void init() override;

    void setLeftSpeed(float speed);
    void setRightSpeed(float speed);

    // Added the "override" keyword so it fulfills the Contract!
    void drive(float leftSpeed, float rightSpeed) override;
    void stop() override;
};