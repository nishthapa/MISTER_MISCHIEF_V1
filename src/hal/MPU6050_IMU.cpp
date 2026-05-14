#include "hal/MPU6050_IMU.h"
#include "utils/RemoteLogger.h"
#include "config/IMUConfig.h" // <-- Bring in the Config!
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

constexpr int16_t DEFAULT_XGYRO_OFFSET = 0;
constexpr int16_t DEFAULT_YGYRO_OFFSET = 0;
constexpr int16_t DEFAULT_ZGYRO_OFFSET = 0;
constexpr int16_t DEFAULT_ZACCEL_OFFSET = 1688;

constexpr uint8_t ACCEL_AUTO_CALIBRATION_SAMPLES = 6;
constexpr uint8_t GYRO_AUTO_CALIBRATION_SAMPLES = 6;

constexpr uint8_t ATTITUDE_YAW_INDEX = 0;
constexpr uint8_t ATTITUDE_PITCH_INDEX = 1;
constexpr uint8_t ATTITUDE_ROLL_INDEX = 2;

MPU6050 mpu;

// DMP memory
uint8_t fifoBuffer[64]; 
Quaternion q;           
VectorFloat gravity;    
float ypr[3];           

MPU6050_IMU::MPU6050_IMU(int sda, int scl, int interruptPin, uint8_t address) {
    sdaPin = sda; sclPin = scl; intPin = interruptPin; deviceAddr = address;
    
    // Initialize our placeholder compass data
    lastKnownAngles.hasCompass = false;
    lastKnownAngles.compassHeading = 0.0f;
}

bool MPU6050_IMU::init() {
    pinMode(intPin, INPUT);
    Wire.begin(sdaPin, sclPin);
    
    // Dynamic Speed Shift!
    if (IMUConfig::MPU6050_USE_HARDWARE_DMP) { Wire.setClock(100000); } 
    else { Wire.setClock(400000); }

    logger.println("Resetting MPU6050 Internal Registers...");
    mpu.reset();
    delay(200); 

    // =========================================================
    // BRANCH A: HARDWARE DMP BOOT 
    // =========================================================
    if (IMUConfig::MPU6050_USE_HARDWARE_DMP) {
        logger.println("INITIALIZING MPU6050 (DMP MODE)...");
        mpu.initialize();
        delay(50);
        
        if (!mpu.testConnection()) {
            logger.println("CRITICAL: MPU6050 connection failed!");
            return false;
        }
        
        logger.println("Uploading Firmware to Digital Motion Processor (DMP)...");
        uint8_t devStatus = 1; int dmpAttempts = 0;

        while (devStatus != 0 && dmpAttempts < 10) {
            devStatus = mpu.dmpInitialize();
            if (devStatus == 0) {
                logger.println("DMP Firmware Upload SUCCESS!");
                break; 
            } else {
                logger.printf("DMP Upload failed (code %d). Retrying...\n", devStatus);
                delay(100);
                dmpAttempts++;
            }
        }

        if (devStatus != 0) return false; 

        mpu.setXGyroOffset(DEFAULT_XGYRO_OFFSET); mpu.setYGyroOffset(DEFAULT_YGYRO_OFFSET);
        mpu.setZGyroOffset(DEFAULT_ZGYRO_OFFSET); mpu.setZAccelOffset(DEFAULT_ZACCEL_OFFSET);
        mpu.CalibrateAccel(ACCEL_AUTO_CALIBRATION_SAMPLES); mpu.CalibrateGyro(GYRO_AUTO_CALIBRATION_SAMPLES);
        mpu.setDMPEnabled(true);
        return true;
    } 
    // =========================================================
    // BRANCH B: RAW DATA BYPASS (Lightning Fast Boot)
    // =========================================================
    else {
        logger.println("DMP Bypassed. Initializing Raw Data Mode...");
        mpu.initialize();
        delay(50);

        if (!mpu.testConnection()) {
            logger.println("CRITICAL: MPU6050 connection failed!");
            return false;
        }

        mpu.setXGyroOffset(DEFAULT_XGYRO_OFFSET); mpu.setYGyroOffset(DEFAULT_YGYRO_OFFSET);
        mpu.setZGyroOffset(DEFAULT_ZGYRO_OFFSET); mpu.setZAccelOffset(DEFAULT_ZACCEL_OFFSET);

        lastUpdateTime = micros();
        logger.println("Raw Mode Initialized Successfully!");
        return true;
    }
}

FusedAngles MPU6050_IMU::getAngles() {
    if (IMUConfig::MPU6050_USE_HARDWARE_DMP) {
        if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            lastKnownAngles.yaw   = ypr[ATTITUDE_YAW_INDEX] * 180 / M_PI;
            lastKnownAngles.pitch = ypr[ATTITUDE_PITCH_INDEX] * 180 / M_PI;
            lastKnownAngles.roll  = ypr[ATTITUDE_ROLL_INDEX] * 180 / M_PI;
        }
    } 
    else {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        unsigned long currentTime = micros();
        float dt = (currentTime - lastUpdateTime) / 1000000.0f;
        lastUpdateTime = currentTime;

        // Convert Gyro to Degrees Per Second 
        float gyroRollRate  = gx / 131.0f;
        float gyroPitchRate = gy / 131.0f;
        float gyroYawRate   = gz / 131.0f;

        // Calculate Absolute Angles from Accelerometer
        float accelRoll  = atan2(ay, az) * 180.0f / M_PI;
        float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;

        // FUSION ALGORITHM
        compRoll  = IMUConfig::COMP_FILTER_ALPHA * (compRoll + gyroRollRate * dt) + (1.0f - IMUConfig::COMP_FILTER_ALPHA) * accelRoll;
        compPitch = IMUConfig::COMP_FILTER_ALPHA * (compPitch + gyroPitchRate * dt) + (1.0f - IMUConfig::COMP_FILTER_ALPHA) * accelPitch;
        compYaw  += gyroYawRate * dt; 

        lastKnownAngles.roll  = compRoll;
        lastKnownAngles.pitch = compPitch;
        lastKnownAngles.yaw   = compYaw;
    }
    
    // Explicitly enforce that we do not have a compass on this hardware
    lastKnownAngles.hasCompass = false;
    lastKnownAngles.compassHeading = 0.0f;
    
    return lastKnownAngles;
}