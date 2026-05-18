#include "hal/hardware/imu/MPU6050_IMU.h"
#include "utils/RemoteLogger.h"
#include "config/SensorConfig.h" 
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

/* <pre>
 *          |   ACCELEROMETER    |           GYROSCOPE
 * DLPF_CFG | Bandwidth | Delay  | Bandwidth | Delay  | Sample Rate
 * ---------+-----------+--------+-----------+--------+-------------
 * 0        | 260Hz     | 0ms    | 256Hz     | 0.98ms | 8kHz
 * 1        | 184Hz     | 2.0ms  | 188Hz     | 1.9ms  | 1kHz
 * 2        | 94Hz      | 3.0ms  | 98Hz      | 2.8ms  | 1kHz
 * 3        | 44Hz      | 4.9ms  | 42Hz      | 4.8ms  | 1kHz
 * 4        | 21Hz      | 8.5ms  | 20Hz      | 8.3ms  | 1kHz
 * 5        | 10Hz      | 13.8ms | 10Hz      | 13.4ms | 1kHz
 * 6        | 5Hz       | 19.0ms | 5Hz       | 18.6ms | 1kHz
 * 7        |   -- Reserved --   |   -- Reserved --   | Reserved
 * </pre>
 */
// Gyro LPF settings (Mechanical Vibration Filter)
// 0 = 260Hz (Jittery), 3 = 42Hz (Smooth), 4 = 20Hz (Buttery
constexpr uint8_t IMU_GYRO_LOW_PASS_FILTER_260HZ = 0;
constexpr uint8_t IMU_GYRO_LOW_PASS_FILTER_188HZ = 1;
constexpr uint8_t IMU_GYRO_LOW_PASS_FILTER_98HZ = 2;
constexpr uint8_t IMU_GYRO_LOW_PASS_FILTER_42HZ = 3;
constexpr uint8_t IMU_GYRO_LOW_PASS_FILTER_20HZ = 4;

MPU6050 mpu;

// DMP memory
uint8_t fifoBuffer[64]; 
Quaternion q;           
VectorFloat gravity;    
float ypr[3];           

MPU6050_IMU::MPU6050_IMU(int sda, int scl, int interruptPin, uint8_t address) {
    sdaPin = sda; sclPin = scl; intPin = interruptPin; deviceAddr = address;
    
    // Initialize our compass data
    lastKnownAngles.hasCompass = IMUConfig::HAS_COMPASS;
    lastKnownAngles.compassHeading = 0.0f;

    filter = new MadgwickFilter(IMUConfig::MADGWICK_BETA);
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

        // === THE SILICON NOISE FILTER ===
        // The DMP used to do this automatically. We must manually tell the MPU6050
        // to filter out high-frequency mechanical chassis vibrations.
        // 0 = 260Hz (Jittery), 3 = 42Hz (Smooth), 4 = 20Hz (Buttery)
        // mpu.setDLPFMode(4);
        mpu.setDLPFMode(IMU_GYRO_LOW_PASS_FILTER_20HZ);

        // NOTE: Manual offset clears removed here to preserve the MPU6050's hardware factory trim!

        lastUpdateTime = micros();
        logger.println("Raw Mode Initialized Successfully!");
        
        // Auto-trigger the silent background calibration sequence on boot
        calibrateGyro();
        
        return true;
    }
}

// ==========================================
// CLI EXECUTION
// ==========================================
void MPU6050_IMU::calibrateGyro() {
    logger.println("\n[IMU] GYROSCOPE CALIBRATION STARTED. KEEP HARDWARE STILL FOR 5 SECONDS...");
    calibratingGyro = true;
    calibrationSamples = 0;
    sumX = 0; sumY = 0; sumZ = 0;
}

void MPU6050_IMU::calibrateAccel() {
    logger.println("\n[IMU] ACCELEROMETER CALIBRATION STARTED. KEEP HARDWARE PERFECTLY LEVEL FOR 5 SECONDS...");
    calibratingAccel = true;
    accelCalibSamples = 0;
    sumAccelX = 0; sumAccelY = 0; sumAccelZ = 0;
}

void MPU6050_IMU::calibrateMag() {
    logger.println("\n[IMU WARNING] The MPU6050 does not have a compass! Command ignored.");
}

// ==========================================
// THE PHYSICS ENGINE
// ==========================================
FusedAngles MPU6050_IMU::getAngles() {
    
    if (IMUConfig::MPU6050_USE_HARDWARE_DMP) {
        // --- DMP HARDWARE ROUTE ---
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
        // --- MADGWICK SOFTWARE ROUTE ---
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // ----------------------------------------------------
        // STATE MACHINE A: ACCELEROMETER CALIBRATION
        // ----------------------------------------------------
        if (calibratingAccel) {
            sumAccelX += ax; sumAccelY += ay; sumAccelZ += az;
            accelCalibSamples++;
            
            if (accelCalibSamples >= IMUConfig::ACCEL_CALIBRATION_SAMPLES) {
                accelBiasX = (float)sumAccelX / (float)IMUConfig::ACCEL_CALIBRATION_SAMPLES;
                accelBiasY = (float)sumAccelY / (float)IMUConfig::ACCEL_CALIBRATION_SAMPLES;
                
                // Gravity Math: Z-axis should be experiencing exactly 1.0G. 
                // We subtract 1.0G from the average to find the true sensor error/bias!
                accelBiasZ = ((float)sumAccelZ / (float)IMUConfig::ACCEL_CALIBRATION_SAMPLES) - IMUConfig::ACCEL_SCALE_FACTOR;
                
                calibratingAccel = false;
                logger.printf("[IMU] Accel Calibrated! Biases: X:%.1f Y:%.1f Z:%.1f\n", accelBiasX, accelBiasY, accelBiasZ);
                lastUpdateTime = micros(); // Prevent time-travel explosion
            }
            return lastKnownAngles; 
        }

        // ----------------------------------------------------
        // STATE MACHINE B: GYROSCOPE CALIBRATION
        // ----------------------------------------------------
        if (calibratingGyro) {
            sumX += gx; sumY += gy; sumZ += gz;
            calibrationSamples++;
            
            //if (calibrationSamples >= 500) {
            if (calibrationSamples >= IMUConfig::GYRO_CALIBRATION_SAMPLES) {
                gyroBiasX = (float)sumX / (float)IMUConfig::GYRO_CALIBRATION_SAMPLES;
                gyroBiasY = (float)sumY / (float)IMUConfig::GYRO_CALIBRATION_SAMPLES;
                gyroBiasZ = (float)sumZ / (float)IMUConfig::GYRO_CALIBRATION_SAMPLES;
                calibratingGyro = false;
                logger.printf("[IMU] Gyro Calibrated! Biases: X:%.1f Y:%.1f Z:%.1f\n", gyroBiasX, gyroBiasY, gyroBiasZ);
                lastUpdateTime = micros();
            }
            return lastKnownAngles; 
        }

        // 1. Calculate DT
        unsigned long currentTime = micros();
        float dt = (currentTime - lastUpdateTime) / 1000000.0f;
        lastUpdateTime = currentTime;

        // 2. Subtract Gyro Bias & Convert to Radians/sec
        float gx_rad = ((float)gx - gyroBiasX) * (M_PI / (180.0f * IMUConfig::GYRO_SCALE_FACTOR));
        float gy_rad = ((float)gy - gyroBiasY) * (M_PI / (180.0f * IMUConfig::GYRO_SCALE_FACTOR));
        float gz_rad = ((float)gz - gyroBiasZ) * (M_PI / (180.0f * IMUConfig::GYRO_SCALE_FACTOR));
        
        if (abs(gz_rad) < IMUConfig::GYRO_DEADBAND_RAD_S) gz_rad = 0.0f; 

        // 3. Subtract Accel Bias
        float ax_cal = (float)ax - accelBiasX;
        float ay_cal = (float)ay - accelBiasY;
        float az_cal = (float)az - accelBiasZ;

        // 4. Feed the Unified Math Engine! 
        filter->compute(gx_rad, gy_rad, gz_rad, ax_cal, ay_cal, az_cal, 0.0f, 0.0f, 0.0f, dt, IMUConfig::HAS_COMPASS);

        // --- NEW: Calculate Total G-Force ---
        float accelMag = sqrt((ax_cal * ax_cal) + (ay_cal * ay_cal) + (az_cal * az_cal));
        lastKnownAngles.gForce = accelMag / IMUConfig::ACCEL_SCALE_FACTOR;

        // 5. Update Memory
        lastKnownAngles.roll  = filter->getRoll();
        lastKnownAngles.pitch = filter->getPitch();
        lastKnownAngles.yaw   = filter->getYaw();
    }
    
    return lastKnownAngles;
}