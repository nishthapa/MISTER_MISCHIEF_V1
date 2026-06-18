#ifndef PACKET_ID_REGISTRY_H
#define PACKET_ID_REGISTRY_H

#include <cstdint>

namespace Comms {

    // =========================================================================
    // 1. MESSAGE / PAYLOAD IDs (What data is being transmitted over the wire)
    // =========================================================================
    enum class MsgId : uint8_t {
        // Core Physics & Motion
        ATTITUDE            = 101,  // Pitch, Roll, Yaw (floats)
        IMU_RAW             = 102,  // Accel X/Y/Z, Gyro X/Y/Z (ints/floats)
        KINEMATICS          = 103,  // Linear velocity, Angular velocity, Heading
        ACTUATORS           = 104,  // Left Motor PWM, Right Motor PWM, Current draw per motor

        // Sensors
        DISTANCE_SONAR      = 110,  // Raw distance, filtered distance, ping success rate
        ANALOG_POWER        = 111,  // Battery Voltage (mV), Current (mA), Power (mW)

        // Control Loops
        PID_ATTITUDE_DEBUG  = 120,  // Target vs Actual, P/I/D contributions for balancing
        PID_DRIVE_DEBUG     = 121,  // Target vs Actual for tracking/driving

        // System, State & Link
        SYSTEM_STATUS       = 130,  // Loop time (us), Free heap memory, Core temperature, HealthBitmask
        ROBOT_STATE         = 131,  // Current Global Mode, Active Mood
        NETWORK_LINK        = 132,  // Wi-Fi RSSI (int8_t), BLE RSSI (int8_t)
        EVENT_STATE         = 135,  // Semantic latches, Dizzy bars, Frustration levels
        PERCEPTION_METRICS  = 136,  // Real-time math derivatives for tuning (latches + energies)
        TRANSIENT_ALERTS    = 140   // Human readable text logs (serial.println() mirror)
    };

    // =========================================================================
    // 2. ROBOT GLOBAL MODES (Maps exactly to your behavior system state)
    // =========================================================================
    enum class RobotMode : uint8_t {
        DEEP_SLEEP          = 0,
        TELEOP              = 1,
        NORMAL_DRIVING      = 2,
        OBSTACLE_AVOIDANCE  = 3,
        MAINTAIN_DISTANCE   = 4,
        COMPASS_LOCK        = 5,
        DIZZY               = 6,
        AUTO_TUNE           = 7,
        DIAGNOSTICS         = 8
    };

    // =========================================================================
    // 3. ROBOT MOODS (For expressive behavioral feedback)
    // =========================================================================
    enum class RobotMood : uint8_t {
        IDLE                = 0,
        CURIOUS             = 1,
        CAUTIOUS            = 2,
        PANICKED            = 3,
        CONFUSED            = 4,
        AGGRESSIVE          = 5
    };

    // =========================================================================
    // 4. HARDWARE HEALTH BITMASK SHIFTS (Ongoing system status flags)
    // Used to pack the state of all components into a single 16-bit field
    // =========================================================================
    namespace HealthBit {
        // FREERTOS STATUS 
        constexpr uint16_t FREE_RTOS_ALIVE      = (1 << 0);  // Watchdog happy, tasks executing within deadline
        
        // SENSOR BUSES
        constexpr uint16_t I2C_BUS_OK           = (1 << 1);  // Main I2C bus clear of lockups
        constexpr uint16_t SPI_BUS_OK           = (1 << 2);  // Auxiliary SPI bus stable

        // SENSOR HEALTH
        constexpr uint16_t IMU_OK               = (1 << 3);  // IMU alive and calibrating/running
        constexpr uint16_t MAG_OK               = (1 << 4);  // Magnetometer ok/available
        constexpr uint16_t BARO_OK              = (1 << 5);  // Barometer ok/available
        constexpr uint16_t SONAR_OK             = (1 << 6);  // HCSR04 sending and receiving echoes
        constexpr uint16_t GPS_OK               = (1 << 7);  // GPS ok/available
        // constexpr uint16_t TOF_OK               = (1 << 8);  // Time of Flight available/ok
        constexpr uint16_t CLIFF_IR_OK          = (1 << 8);  // Infrared cliff sensor(s) available/ok
        constexpr uint16_t LIDAR_OK             = (1 << 9); // LIDAR available/ok
       
        // OTHER HARDWARE HEALTH
        constexpr uint16_t MOTOR_DRIVER_OK      = (1 << 10);  // XY160D driver pins/timers responding
        constexpr uint16_t POWER_MONITOR_OK     = (1 << 11);  // Battery/INA226 communications healthy

        // SELF RIGHTING BITS // These dont belong here!
        // constexpr uint16_t ROBOT_FLIPPED        = (1 << 13);  // Indicator or whether robot is flipped or on its sides
        constexpr uint16_t ANTIFLIP_SERVO_OK    = (1 << 12);  // Servo to self right if the robot flips available/ok
        
        // NETWORK STATES
        constexpr uint16_t WIFI_CONNECTED       = (1 << 13);  // <--- NEW: Wi-Fi connected to router
        constexpr uint16_t BLE_CONNECTED        = (1 << 14);  // Active central device connected
        constexpr uint16_t USB_CONNECTED        = (1 << 15);  // USB connected
    }

    // =========================================================================
    // 5. TRANSIENT ALERT/ERROR CODES (Instantaneous micro-fault blips)
    // Pushed to your AlertQueue whenever an exceptional single event happens
    // =========================================================================
    enum class AlertCode : uint8_t {
        NONE                    = 0,
        
        // Bus & Hardware Blips
        //I2C_TIMEOUT             = 10,  // I2C bus temporarily dropped a packet
        //I2C_NACK                = 11,  // Device didn't acknowledge address
        //SPI_CRC_ERROR           = 20,  // SPI corrupted data detected
        //SPI_TIMEOUT             = 21,  // SPI bus timed out during transmission
        
        // Sensor Anomalies
        //IMU_DATA_STALE          = 30,  // FIFO buffer didn't update in time
        //IMU_FIFO_OVERFLOW       = 31,  // MPU6050 internal memory overflowed
        SONAR_TIMEOUT           = 40,  // Echo pin never returned high
        SONAR_OUT_OF_BOUNDS     = 41,  // Distance jumped drastically outside physical limits
        
        // Motor & Kinematics Watchdogs
        MOTOR_STALL_DETECTED    = 50,  // High current draw without robot movement
        KINEMATICS_IMPOSSIBLE   = 51,  // Requested twist command exceeds physical constraints
        
        // Power/Battery Safeguards
        BATTERY_LOW_WARNING     = 60,  // Voltage below nominal warning threshold
        BATTERY_CRITICAL_FLOP   = 61,  // Critical voltage dip during motor surge
        CURRENT_SPIKE_LIMIT     = 62,  // Hard overcurrent protection tripped
        
        // Connectivity & Link Safeguards
        // WIFI_DISCONNECTED       = 70,  // Unexpected drop from the Wi-Fi router
        // BLE_CLIENT_DROPPED      = 71,  // Android app disconnected abruptly
        // LINK_QUALITY_CRITICAL   = 72,  // RSSI dropped below safe control threshold (e.g., -90dBm)

        // Software OS Health
        TELEMETRY_QUEUE_FULL    = 80,  // Network buffers filling up faster than link can stream
        LOOP_TIME_EXCEEDED      = 81   // Physics loop missed its execution window deadline
    };

    // =========================================================================
    // 6. COMMAND / REQUEST IDs (Inbound messages from the Android App)
    // =========================================================================
    enum class CmdId : uint8_t {
        SET_MODE            = 1,   // Change operational mode (payload: RobotMode)
        TRIGGER_CALIBRATION = 2,   // Re-zero IMU gyro offsets

        TUNING_UPDATE_P     = 10,  // Update Proportional gain (payload: float)
        TUNING_UPDATE_I     = 11,  // Update Integral gain (payload: float)
        TUNING_UPDATE_D     = 12,  // Update Derivative gain (payload: float)

        CLEAR_ALERT_QUEUE   = 99   // Resets the error counters on the robot
    };

} // namespace Comms

#endif // PACKET_ID_REGISTRY_H