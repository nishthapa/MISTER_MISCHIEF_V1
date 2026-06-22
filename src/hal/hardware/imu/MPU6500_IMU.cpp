#include "hal/hardware/imu/MPU6500_IMU.h"
#include "utils/RemoteLogger.h"
#include "config/SensorConfig.h"
#include "config/SystemConfig.h"
#include "config/ConfigurationManager.h"
#include <Wire.h>

MPU6500_IMU::MPU6500_IMU(int sda, int scl, int ncs, uint8_t address) {
    sdaPin = sda; 
    sclPin = scl; 
    ncsPin = ncs; 
    deviceAddr = address;
    
    // If a valid Chip Select pin is provided, arm the SPI flags
    //useSPI = (ncsPin >= 0);
    useSPI = false; // Hardcode to I2C for now since the SPI code is untested and DMP is not supported on SPI mode in the xreef library
    
    // Hardcode to false: GY-91 clone drops the AK8963 compass
    lastKnownAngles.hasCompass = false; 
    lastKnownAngles.compassHeading = 0.0f;
    lastKnownAngles.gForce = 1.0f;

    filter = new MadgwickFilter(SysConfig.MADGWICK_FILTER_BETA);
}

bool MPU6500_IMU::init() {
    logger.println("Initializing MPU6500...");

    if (!useSPI) {
        // =======================================
        // I2C MODE 
        // =======================================
        Wire.begin(sdaPin, sclPin);
        Wire.setClock(SystemConfig::I2C_CLOCK_SPEED_HZ);

        // Never wait more than 50ms for a stuck I2C bus!
        Wire.setTimeOut(50);

        if (imu.begin() != INV_SUCCESS) {
            logger.println("CRITICAL: MPU6500 I2C connection failed!");
            return false;
        }

        // Attempt to boot the DMP firmware onto the chip using the xreef library
        //if (imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL, IMUConfig::IMU_DMP_SAMPLE_RATE_HZ) == INV_SUCCESS) {
        if (imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_SEND_RAW_ACCEL, IMUConfig::IMU_DMP_SAMPLE_RATE_HZ) == INV_SUCCESS) {
            logger.println("Hardware DMP Booted Successfully! Quaternion math offloaded.");
            usingDMP = true;
        } else {
            logger.println("DMP Boot Failed over I2C. Falling back to Raw Data + Madgwick Filter.");
            usingDMP = false;
        }
    } else {
        // =======================================
        // SPI MODE (Future Upgrade)
        // =======================================
        logger.println("SPI Mode Selected. DMP is unsupported. Forcing Raw Data Madgwick bypass.");
        usingDMP = false;
        
        // Manual SPI instantiation based on the xreef library structure
        // imu.beginSPI(ncsPin); 
    }

    // --- CONFIGURE RAW SENSORS (If DMP is offline or we are on SPI) ---
    if (!usingDMP) {
        imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        imu.setGyroFSR(2000);
        imu.setAccelFSR(2);
        imu.setSampleRate(IMUConfig::IMU_RAW_SAMPLE_RATE_HZ);
        imu.setLPF(42); // 42Hz digital low pass filter to kill mechanical chassis vibrations
    }

    lastUpdateTime = micros();
    return true;
}

float MPU6500_IMU::getTemperature() {
    return 0.0f; // Handled by BMP280 externally
}

void MPU6500_IMU::calibrateGyro() {
    logger.println("\n[IMU] GYROSCOPE CALIBRATION STARTED.");
    // Handled internally by DMP or requires manual offset calculation in raw mode
}

void MPU6500_IMU::calibrateAccel() {
    logger.println("\n[IMU] ACCELEROMETER CALIBRATION STARTED.");
}

void MPU6500_IMU::calibrateMag() {
    logger.println("\n[IMU WARNING] Compass missing from hardware. Command ignored.");
}

FusedAngles MPU6500_IMU::getAngles() {
    unsigned long currentTime = micros();
    float dt = (currentTime - lastUpdateTime) / 1000000.0f;
    lastUpdateTime = currentTime;

    if (usingDMP) {
        // =======================================
        // ROUTE A: HARDWARE DMP (Polling)
        // =======================================
        if (imu.fifoAvailable()) {
            if (imu.dmpUpdateFifo() == INV_SUCCESS) {
                imu.computeEulerAngles(false); 
                
                // Calculate Gs using the xreef library calculations
                float ax_g = imu.calcAccel(imu.ax);
                float ay_g = imu.calcAccel(imu.ay);
                float az_g = imu.calcAccel(imu.az);
                
                lastKnownAngles.gForce = sqrt((ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g));

                // lastKnownAngles.roll  = imu.roll  * (SysConfig.IMU_INVERT_ROLL ? -1.0f : 1.0f);
                // lastKnownAngles.pitch = imu.pitch * (SysConfig.IMU_INVERT_PITCH ? -1.0f : 1.0f);
                // lastKnownAngles.yaw   = imu.yaw   * (SysConfig.IMU_INVERT_YAW ? -1.0f : 1.0f);

                // FIX: Convert Radians to Degrees by multiplying by (180.0f / M_PI)
                lastKnownAngles.roll  = (imu.roll  * (180.0f / M_PI)) * (SysConfig.IMU_INVERT_ROLL ? -1.0f : 1.0f);
                lastKnownAngles.pitch = (imu.pitch * (180.0f / M_PI)) * (SysConfig.IMU_INVERT_PITCH ? -1.0f : 1.0f);
                lastKnownAngles.yaw   = (imu.yaw   * (180.0f / M_PI)) * (SysConfig.IMU_INVERT_YAW ? -1.0f : 1.0f);
            }
        }
    } else {
        // =======================================
        // ROUTE B: RAW DATA + MADGWICK (I2C/SPI Fallback)
        // =======================================
        if (imu.dataReady()) {
            imu.update(UPDATE_ACCEL | UPDATE_GYRO);
            
            float ax = filterAx.addSample(imu.calcAccel(imu.ax));
            float ay = filterAy.addSample(imu.calcAccel(imu.ay));
            float az = filterAz.addSample(imu.calcAccel(imu.az));

            float gx_rad = filterGx.addSample(imu.calcGyro(imu.gx)) * (M_PI / 180.0f);
            float gy_rad = filterGy.addSample(imu.calcGyro(imu.gy)) * (M_PI / 180.0f);
            float gz_rad = filterGz.addSample(imu.calcGyro(imu.gz)) * (M_PI / 180.0f);

            float currentGForce = sqrt((ax * ax) + (ay * ay) + (az * az));
            lastKnownAngles.gForce = currentGForce;

            // G-Force Gatekeeper logic
            if (currentGForce > SysConfig.GFORCE_LIFT_UP_THRESHOLD || currentGForce < SysConfig.GFORCE_LIFT_DOWN_THRESHOLD) {
                filter->compute(gx_rad, gy_rad, gz_rad, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, dt, false);
            } else {
                filter->compute(gx_rad, gy_rad, gz_rad, ax, ay, az, 0.0f, 0.0f, 0.0f, dt, false);
            }

            lastKnownAngles.roll  = filter->getRoll()  * (SysConfig.IMU_INVERT_ROLL ? -1.0f : 1.0f);
            lastKnownAngles.pitch = filter->getPitch() * (SysConfig.IMU_INVERT_PITCH ? -1.0f : 1.0f);
            lastKnownAngles.yaw   = filter->getYaw()   * (SysConfig.IMU_INVERT_YAW ? -1.0f : 1.0f);
        }
    }
    
    return lastKnownAngles;
}