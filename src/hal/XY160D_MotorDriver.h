#pragma once

class XY160D_MotorDriver {
    private:
    // Internal memory to hold the pin assignments
    int leftFwdPin;
    int leftRevPin;
    int rightFwdPin;
    int rightRevPin;

public:
    // The new parameterized constructor
    XY160D_MotorDriver(int leftFwd, int leftRev, int rightFwd, int rightRev);

    // Configures the ESP32 PWM timers and attaches the pins
    void init();

    // Accepts a speed percentage from -100 (full reverse) to 100 (full forward).
    // Automatically scales to your safe 3V PWM limit.
    void setLeftSpeed(int speed);
    void setRightSpeed(int speed);

    // Convenience command to drive both tracks simultaneously
    void drive(int leftSpeed, int rightSpeed);

    // Hard emergency stop (grounds all H-bridge inputs)
    void stop();
};