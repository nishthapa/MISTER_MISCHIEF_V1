#include "hal/hardware/motordriver/XY160D_MotorDriver.h"
#include "config/MotorDriverConfig.h"
#include "config/PinConfig.h" // For the default pin assignments
#include "config/ConfigurationManager.h"

// The Core 2.x API requires routing pins through specific hardware channels
#include "config/ChannelRegistry.h" // --- ESP32 Hardware PWM Channels ---

#include <Arduino.h>

// 1. Store the incoming pins into the object's private memory
XY160D_MotorDriver::XY160D_MotorDriver(int lf, int lr, int rf, int rr) {
    leftFwdPin = lf;
    leftRevPin = lr;
    rightFwdPin = rf;
    rightRevPin = rr;
}

void XY160D_MotorDriver::init() {
    // 1. Configure the 4 hardware channels with your frequency and resolution
    // (These CH_ variables are still fine, they come from ChannelRegistry.h)
    ledcSetup(ChannelRegistry::CH_MOTOR_LEFT_FWD, MotorDriverConfig::PWM_FREQ, MotorDriverConfig::PWM_RES);
    ledcSetup(ChannelRegistry::CH_MOTOR_LEFT_REV, MotorDriverConfig::PWM_FREQ, MotorDriverConfig::PWM_RES);
    ledcSetup(ChannelRegistry::CH_MOTOR_RIGHT_FWD, MotorDriverConfig::PWM_FREQ, MotorDriverConfig::PWM_RES);
    ledcSetup(ChannelRegistry::CH_MOTOR_RIGHT_REV, MotorDriverConfig::PWM_FREQ, MotorDriverConfig::PWM_RES);

    // 2. Attach the internally stored pins to the channels
    // DO NOT use PIN_MOTOR_..., use the private variables!
    ledcAttachPin(leftFwdPin, ChannelRegistry::CH_MOTOR_LEFT_FWD);
    ledcAttachPin(leftRevPin, ChannelRegistry::CH_MOTOR_LEFT_REV);
    ledcAttachPin(rightFwdPin, ChannelRegistry::CH_MOTOR_RIGHT_FWD);
    ledcAttachPin(rightRevPin, ChannelRegistry::CH_MOTOR_RIGHT_REV);
    
    // Ensure motors are dead silent and locked on boot
    stop(); 
}

void XY160D_MotorDriver::setLeftSpeed(float speed) {
    speed = constrain(speed, -100.0f, 100.0f);
    // If speed is > 0, map it starting from the bare minimum PWM required to move the tracks!
    int pwmValue = map(abs(speed), 0.1f, 100.0f, Config.MOTOR_MIN_PWM, MotorDriverConfig::MAX_DUTY);

    // Note: Core 2.x ledcWrite targets the CHANNEL, not the physical PIN
    if (speed > 0) {
        ledcWrite(ChannelRegistry::CH_MOTOR_LEFT_FWD, pwmValue);
        ledcWrite(ChannelRegistry::CH_MOTOR_LEFT_REV, 0);       
    } else if (speed < 0) {
        ledcWrite(ChannelRegistry::CH_MOTOR_LEFT_FWD, 0);       
        ledcWrite(ChannelRegistry::CH_MOTOR_LEFT_REV, pwmValue);
    } else {
        ledcWrite(ChannelRegistry::CH_MOTOR_LEFT_FWD, 0);
        ledcWrite(ChannelRegistry::CH_MOTOR_LEFT_REV, 0);
    }
}

void XY160D_MotorDriver::setRightSpeed(float speed) {
    speed = constrain(speed, -100.0f, 100.0f);
    // If speed is > 0, map it starting from the bare minimum PWM required to move the tracks!
    int pwmValue = map(abs(speed), 0.1f, 100.0f, Config.MOTOR_MIN_PWM, MotorDriverConfig::MAX_DUTY);

    if (speed > 0) {
        ledcWrite(ChannelRegistry::CH_MOTOR_RIGHT_FWD, pwmValue);
        ledcWrite(ChannelRegistry::CH_MOTOR_RIGHT_REV, 0);
    } else if (speed < 0) {
        ledcWrite(ChannelRegistry::CH_MOTOR_RIGHT_FWD, 0);
        ledcWrite(ChannelRegistry::CH_MOTOR_RIGHT_REV, pwmValue);
    } else {
        ledcWrite(ChannelRegistry::CH_MOTOR_RIGHT_FWD, 0);
        ledcWrite(ChannelRegistry::CH_MOTOR_RIGHT_REV, 0);
    }
}

void XY160D_MotorDriver::drive(float leftSpeed, float rightSpeed) {
    setLeftSpeed(leftSpeed);
    setRightSpeed(rightSpeed);
}

void XY160D_MotorDriver::stop() {
    drive(0.0f, 0.0f);
}