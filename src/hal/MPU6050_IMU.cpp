#include "hal/MPU6050_IMU.h"
#include <Wire.h>

// --- THE QUARANTINED LIBRARIES ---
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

constexpr uint32_t I2C_COMM_FREQUENCY = 100000; // 100kHz I2C for fast DMP packet transfers and perfboard stability

// These are baseline offsets. We will calibrate these specifically for our chip later (16-bit signed integers)
constexpr int16_t DEFAULT_XGYRO_OFFSET = 0;
constexpr int16_t DEFAULT_YGYRO_OFFSET = 0;
constexpr int16_t DEFAULT_ZGYRO_OFFSET = 0;
constexpr int16_t DEFAULT_ZACCEL_OFFSET = 1688;

// Constants for Quick Auto-Calibration Routine
constexpr uint8_t ACCEL_AUTO_CALIBRATION_SAMPLES = 6;
constexpr uint8_t GYRO_AUTO_CALIBRATION_SAMPLES = 6;

// Euler Attitude Array Indexes
constexpr uint8_t ATTITUDE_YAW_INDEX = 0;
constexpr uint8_t ATTITUDE_PITCH_INDEX = 1;
constexpr uint8_t ATTITUDE_ROLL_INDEX = 2;

// The hidden 3rd-party Electronic Cats MPU6050 Library object
MPU6050 mpu;

// DMP working variables (Hidden from main.cpp)
uint8_t fifoBuffer[64]; // FIFO storage buffer for the DMP packet
Quaternion q;           // [w, x, y, z] quaternion container
VectorFloat gravity;    // [x, y, z] gravity vector
float ypr[3];           // [yaw, pitch, roll] container

MPU6050_IMU::MPU6050_IMU(int sda, int scl, int interruptPin, uint8_t address) {
    sdaPin = sda;
    sclPin = scl;
    intPin = interruptPin;
    deviceAddr = address;
}

bool MPU6050_IMU::init() {
    // The library defaults the INT pin to Active HIGH
    pinMode(intPin, INPUT);
    
    Wire.begin(sdaPin, sclPin);
    Wire.setClock(I2C_COMM_FREQUENCY);

    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.println("CRITICAL: MPU6050 connection failed!");
        return false;
    }

    Serial.println("Initializing Digital Motion Processor (DMP)...");
    uint8_t devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(DEFAULT_XGYRO_OFFSET);
    mpu.setYGyroOffset(DEFAULT_YGYRO_OFFSET);
    mpu.setZGyroOffset(DEFAULT_ZGYRO_OFFSET);
    mpu.setZAccelOffset(DEFAULT_ZACCEL_OFFSET);

    // devStatus == 0 means the firmware blob successfully loaded
    if (devStatus == 0) {
        // Run a quick auto-calibration to zero out minor noise
        mpu.CalibrateAccel(ACCEL_AUTO_CALIBRATION_SAMPLES);
        mpu.CalibrateGyro(GYRO_AUTO_CALIBRATION_SAMPLES);
        
        // Turn the DMP on!
        mpu.setDMPEnabled(true);
        return true;
    } else {
        Serial.printf("DMP Initialization failed (code %d)\n", devStatus);
        return false;
    }
}

bool MPU6050_IMU::isDataReady() {
    // The library configures the INT pin as Active HIGH by default
    return (digitalRead(intPin) == HIGH);
}

FusedAngles MPU6050_IMU::getAngles() {
    // REMOVE the isDataReady() check completely!
    // Let the library directly manage and flush the FIFO buffer.
    
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Update our internal memory
        lastKnownAngles.yaw   = ypr[ATTITUDE_YAW_INDEX] * 180 / M_PI;
        lastKnownAngles.pitch = ypr[ATTITUDE_PITCH_INDEX] * 180 / M_PI;
        lastKnownAngles.roll  = ypr[ATTITUDE_ROLL_INDEX] * 180 / M_PI;
    }
    
    // Return the freshest memory
    return lastKnownAngles;
}