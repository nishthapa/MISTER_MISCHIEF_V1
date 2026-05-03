#include "hal/XY160D_MotorDriver.h"
#include "config/MotorPWMConfig.h"

// The Core 2.x API requires routing pins through specific hardware channels
#include "config/ChannelRegistry.h" // --- ESP32 Hardware PWM Channels ---

#include <Arduino.h>

// 1. Store the incoming pins into the object's private memory
XY160D_MotorDriver::XY160D_MotorDriver(int leftFwd, int leftRev, int rightFwd, int rightRev) {
    leftFwdPin = leftFwd;
    leftRevPin = leftRev;
    rightFwdPin = rightFwd;
    rightRevPin = rightRev;
}

void XY160D_MotorDriver::init() {
    // 1. Configure the 4 hardware channels with your frequency and resolution
    // (These CH_ variables are still fine, they come from ChannelRegistry.h)
    ledcSetup(CH_MOTOR_LEFT_FWD, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_MOTOR_LEFT_REV, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_MOTOR_RIGHT_FWD, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_MOTOR_RIGHT_REV, PWM_FREQ, PWM_RESOLUTION);

    // 2. Attach the internally stored pins to the channels
    // DO NOT use PIN_MOTOR_..., use the private variables!
    ledcAttachPin(leftFwdPin, CH_MOTOR_LEFT_FWD);
    ledcAttachPin(leftRevPin, CH_MOTOR_LEFT_REV);
    ledcAttachPin(rightFwdPin, CH_MOTOR_RIGHT_FWD);
    ledcAttachPin(rightRevPin, CH_MOTOR_RIGHT_REV);
    
    // Ensure motors are dead silent and locked on boot
    stop(); 
}

void XY160D_MotorDriver::setLeftSpeed(int speed) {
    speed = constrain(speed, -100, 100);
    int pwmValue = map(abs(speed), 0, 100, 0, PWM_MAX_DUTY_CYCLE);

    // Note: Core 2.x ledcWrite targets the CHANNEL, not the physical PIN
    if (speed > 0) {
        ledcWrite(CH_MOTOR_LEFT_FWD, pwmValue);
        ledcWrite(CH_MOTOR_LEFT_REV, 0);       
    } else if (speed < 0) {
        ledcWrite(CH_MOTOR_LEFT_FWD, 0);       
        ledcWrite(CH_MOTOR_LEFT_REV, pwmValue);
    } else {
        ledcWrite(CH_MOTOR_LEFT_FWD, 0);
        ledcWrite(CH_MOTOR_LEFT_REV, 0);
    }
}

void XY160D_MotorDriver::setRightSpeed(int speed) {
    speed = constrain(speed, -100, 100);
    int pwmValue = map(abs(speed), 0, 100, 0, PWM_MAX_DUTY_CYCLE);

    if (speed > 0) {
        ledcWrite(CH_MOTOR_RIGHT_FWD, pwmValue);
        ledcWrite(CH_MOTOR_RIGHT_REV, 0);
    } else if (speed < 0) {
        ledcWrite(CH_MOTOR_RIGHT_FWD, 0);
        ledcWrite(CH_MOTOR_RIGHT_REV, pwmValue);
    } else {
        ledcWrite(CH_MOTOR_RIGHT_FWD, 0);
        ledcWrite(CH_MOTOR_RIGHT_REV, 0);
    }
}

void XY160D_MotorDriver::drive(int leftSpeed, int rightSpeed) {
    setLeftSpeed(leftSpeed);
    setRightSpeed(rightSpeed);
}

void XY160D_MotorDriver::stop() {
    drive(0, 0);
}