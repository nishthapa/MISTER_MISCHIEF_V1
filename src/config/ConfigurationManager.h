#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "config/FactoryDefaults.h" // <-- Include our new master defaults

struct MasterSettings {
    unsigned long SERIAL_BAUD_RATE = FactoryDefaults::SERIAL_BAUD_RATE;
    // --- Base Settings ---
    float CRUISING_SPEED = FactoryDefaults::CRUISING_SPEED;
    float OBSTACLE_TRIGGER_CM = FactoryDefaults::OBSTACLE_TRIGGER_CM;
    float MAINTAIN_DISTANCE_CM = FactoryDefaults::MAINTAIN_DISTANCE_CM;
    int MOTOR_MIN_PWM = FactoryDefaults::MOTOR_MIN_PWM;

    // --- Obstacle Logic ---
    float OBS_SWEEP_ANGLE = FactoryDefaults::OBS_SWEEP_ANGLE;
    float OBS_SWEEP_SPEED = FactoryDefaults::OBS_SWEEP_SPEED;
    int OBS_SWEEP_PAUSE = FactoryDefaults::OBS_SWEEP_PAUSE;
    float OBS_CLEAR_THRESH = FactoryDefaults::OBS_CLEAR_THRESH;
    float OBS_HYSTERESIS = FactoryDefaults::OBS_HYSTERESIS;

    // --- Obstacle Escape Settings ---
    float OBSTACLE_ESCAPE_BASE_SPEED = FactoryDefaults::OBSTACLE_ESCAPE_BASE_SPEED;
    unsigned long OBSTACLE_BACKUP_DURATION_MS = FactoryDefaults::OBSTACLE_BACKUP_DURATION_MS;
    float OBSTACLE_SWEEP_ANGLE_DEG = FactoryDefaults::OBSTACLE_SWEEP_ANGLE_DEG;
    unsigned long OBSTACLE_SWEEP_TIMEOUT_MS = FactoryDefaults::OBSTACLE_SWEEP_TIMEOUT_MS;
    unsigned long OBSTACLE_ALIGN_TIMEOUT_MS = FactoryDefaults::OBSTACLE_ALIGN_TIMEOUT_MS;
    float OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG = FactoryDefaults::OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG;
    unsigned long OBSTACLE_ESCAPE_DURATION_MS = FactoryDefaults::OBSTACLE_ESCAPE_DURATION_MS;

    // --- Timers ---
    int DIZZY_SPIN_PWM = FactoryDefaults::DIZZY_SPIN_PWM;
    int DIZZY_SPIN_TIME = FactoryDefaults::DIZZY_SPIN_TIME;
    int DIZZY_COOLDOWN = FactoryDefaults::DIZZY_COOLDOWN;
    int SLEEP_TIMEOUT_MS = FactoryDefaults::SLEEP_TIMEOUT_MS;
    float SLEEP_WAKE_G = FactoryDefaults::SLEEP_WAKE_G;

    // --- Compass Lock Settings
    unsigned long COMPASS_LOCK_ENTRY_SETTLE_MS = FactoryDefaults::COMPASS_LOCK_ENTRY_SETTLE_MS;
    unsigned long COMPASS_LOCK_EXIT_SETTLE_MS = FactoryDefaults::COMPASS_LOCK_EXIT_SETTLE_MS;

    // --- Sensors ---
    float IMU_GYRO_DEADBAND = FactoryDefaults::IMU_GYRO_DEADBAND;
    float SONAR_MAX_DIST = FactoryDefaults::SONAR_MAX_DIST;

    // --- IMU ORIENTATION ---
    bool IMU_INVERT_ROLL = FactoryDefaults::IMU_INVERT_ROLL;
    bool IMU_INVERT_PITCH = FactoryDefaults::IMU_INVERT_PITCH;
    bool IMU_INVERT_YAW = FactoryDefaults::IMU_INVERT_YAW;

    // --- Madgwick Filter ---
    float MADGWICK_FILTER_BETA = FactoryDefaults::MADGWICK_FILTER_BETA;

    // --- Automatic PID Tuning ---
    unsigned long AUTOTUNE_START_DELAY_MS = FactoryDefaults::AUTOTUNE_START_DELAY_MS;
    unsigned long AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS = FactoryDefaults::AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS;

    // --- PID: Unified Point Turn ---
    float PID_POINT_P = FactoryDefaults::PID_POINT_P;
    float PID_POINT_I = FactoryDefaults::PID_POINT_I;
    float PID_POINT_D = FactoryDefaults::PID_POINT_D;
    float PID_POINT_LIM = FactoryDefaults::PID_POINT_LIM;
    float PID_POINT_ILIM = FactoryDefaults::PID_POINT_ILIM;
    float PID_POINT_DEAD = FactoryDefaults::PID_POINT_DEAD;

    // --- PID: Unified Arc Turn ---
    float PID_ARC_P = FactoryDefaults::PID_ARC_P;
    float PID_ARC_I = FactoryDefaults::PID_ARC_I;
    float PID_ARC_D = FactoryDefaults::PID_ARC_D;
    float PID_ARC_LIM = FactoryDefaults::PID_ARC_LIM;
    float PID_ARC_ILIM = FactoryDefaults::PID_ARC_ILIM;
    float PID_ARC_DEAD = FactoryDefaults::PID_ARC_DEAD;

    /*
    // --- PID: Heading ---
    float PID_HEADING_P = FactoryDefaults::PID_HEADING_P;
    float PID_HEADING_I = FactoryDefaults::PID_HEADING_I;
    float PID_HEADING_D = FactoryDefaults::PID_HEADING_D;
    float PID_HEADING_LIM = FactoryDefaults::PID_HEADING_LIM;
    float PID_HEADING_ILIM = FactoryDefaults::PID_HEADING_ILIM;
    float PID_HEADING_DEAD = FactoryDefaults::PID_HEADING_DEAD;

    // --- PID: Compass Lock ---
    float PID_COMPASS_P = FactoryDefaults::PID_COMPASS_P;
    float PID_COMPASS_I = FactoryDefaults::PID_COMPASS_I;
    float PID_COMPASS_D = FactoryDefaults::PID_COMPASS_D;
    float PID_COMPASS_LIM = FactoryDefaults::PID_COMPASS_LIM;
    float PID_COMPASS_ILIM = FactoryDefaults::PID_COMPASS_ILIM;
    float PID_COMPASS_DEAD = FactoryDefaults::PID_COMPASS_DEAD;
    */

    // --- PID: Distance ---
    float PID_DIST_P = FactoryDefaults::PID_DIST_P;
    float PID_DIST_I = FactoryDefaults::PID_DIST_I;
    float PID_DIST_D = FactoryDefaults::PID_DIST_D;
    float PID_DIST_LIM = FactoryDefaults::PID_DIST_LIM;
    float PID_DIST_ILIM = FactoryDefaults::PID_DIST_ILIM;
    float PID_DIST_DEAD = FactoryDefaults::PID_DIST_DEAD;

    /*
    // --- PID: Obstacle ---
    float PID_OBSTACLE_P = FactoryDefaults::PID_OBSTACLE_P;
    float PID_OBSTACLE_I = FactoryDefaults::PID_OBSTACLE_I;
    float PID_OBSTACLE_D = FactoryDefaults::PID_OBSTACLE_D;
    float PID_OBSTACLE_LIM = FactoryDefaults::PID_OBSTACLE_LIM;
    float PID_OBSTACLE_ILIM = FactoryDefaults::PID_OBSTACLE_ILIM;
    float PID_OBSTACLE_DEAD = FactoryDefaults::PID_OBSTACLE_DEAD;
    */

    // --- Physics & Handing ---
    float TILT_HANDLING_THRESHOLD = FactoryDefaults::TILT_HANDLING_THRESHOLD;
    float GFORCE_LIFT_UP_THRESHOLD = FactoryDefaults::GFORCE_LIFT_UP_THRESHOLD;
    float GFORCE_LIFT_DOWN_THRESHOLD = FactoryDefaults::GFORCE_LIFT_DOWN_THRESHOLD;
    float LIFT_ENERGY_SPIKE_THRESHOLD = FactoryDefaults::LIFT_ENERGY_SPIKE_THRESHOLD;
    float UPRIGHT_ANGLE_TOLERANCE = FactoryDefaults::UPRIGHT_ANGLE_TOLERANCE;
    float PERFECTLY_STILL_ENERGY = FactoryDefaults::PERFECTLY_STILL_ENERGY;
    float STEADY_HOLD_ENERGY_MAX = FactoryDefaults::STEADY_HOLD_ENERGY_MAX;

    // --- Dizzy & Energy ---
    float DIZZY_ENERGY_DEADBAND = FactoryDefaults::DIZZY_ENERGY_DEADBAND;
    float DIZZY_CHARGE_RATE = FactoryDefaults::DIZZY_CHARGE_RATE;
    float DIZZY_DECAY_RATE = FactoryDefaults::DIZZY_DECAY_RATE;
    float DIZZY_TRIGGER_THRESHOLD = FactoryDefaults::DIZZY_TRIGGER_THRESHOLD;
    float ENERGY_EMA_ALPHA = FactoryDefaults::ENERGY_EMA_ALPHA;
    float ENERGY_EMA_BETA = FactoryDefaults::ENERGY_EMA_BETA;
    unsigned long DIZZY_DURATION_MS = FactoryDefaults::DIZZY_DURATION_MS;

    // --- Mood ---
    float DISTANCE_HOLD_FRUSTRATION_LIMIT = FactoryDefaults::DISTANCE_HOLD_FRUSTRATION_LIMIT;
    float FRUSTRATION_COOLDOWN_RATE = FactoryDefaults::FRUSTRATION_COOLDOWN_RATE;
    float FRUSTRATION_HEATUP_RATE = FactoryDefaults::FRUSTRATION_HEATUP_RATE;

    // --- System & Network ---
    bool BRAIN_ACTIVE = FactoryDefaults::BRAIN_ACTIVE;
    String WIFI_SSID = FactoryDefaults::WIFI_SSID;
    String WIFI_PASSWORD = FactoryDefaults::WIFI_PASSWORD;
    bool WIFI_ACTIVE = FactoryDefaults::WIFI_ACTIVE;
    String BT_NAME = FactoryDefaults::BT_NAME;
    bool BT_ACTIVE = FactoryDefaults::BT_ACTIVE;
    
    bool SERIAL_DEBUG_MASTER = FactoryDefaults::SERIAL_DEBUG_MASTER;
    bool SERIAL_DEBUG_IMU = FactoryDefaults::SERIAL_DEBUG_IMU;   
    bool SERIAL_DEBUG_SONAR = FactoryDefaults::SERIAL_DEBUG_SONAR; 
    bool SERIAL_DEBUG_MOTOR_DRIVER = FactoryDefaults::SERIAL_DEBUG_MOTOR_DRIVER;

    uint8_t DEBUG_ACTIVE = FactoryDefaults::DEBUG_ACTIVE;
    uint8_t DEBUG_USB = FactoryDefaults::DEBUG_USB;
    uint8_t DEBUG_WIFI = FactoryDefaults::DEBUG_WIFI;
    uint8_t DEBUG_BLUETOOTH = FactoryDefaults::DEBUG_BLUETOOTH;
    uint8_t ACTIVE_DEBUG_MODE = FactoryDefaults::ACTIVE_DEBUG_MODE;
};

class ConfigurationManager {
    private:
        Preferences preferences;
        MasterSettings currentSettings;

    public:
        static ConfigurationManager& getInstance() {
            static ConfigurationManager instance;
            return instance;
        }

    // KEYS (E.G. "CRUISE_SPD") SHOULD BE LESS THAN 15 CHARACTERS TO FIT IN PREFERENCES STORAGE
    void init() {
        preferences.begin("mischief", false);

        currentSettings.SERIAL_BAUD_RATE = preferences.getULong("SERIAL_BAUD", FactoryDefaults::SERIAL_BAUD_RATE);

        // --- Base Settings ---
        currentSettings.CRUISING_SPEED = preferences.getFloat("CRUISE_SPD", FactoryDefaults::CRUISING_SPEED);
        currentSettings.OBSTACLE_TRIGGER_CM = preferences.getFloat("OBS_TRIG_CM", FactoryDefaults::OBSTACLE_TRIGGER_CM);
        currentSettings.MAINTAIN_DISTANCE_CM = preferences.getFloat("MAINT_DIST_CM", FactoryDefaults::MAINTAIN_DISTANCE_CM);
        currentSettings.MOTOR_MIN_PWM = preferences.getInt("MOT_MIN", FactoryDefaults::MOTOR_MIN_PWM);

        // --- Obstacle Logic ---
        currentSettings.OBS_SWEEP_ANGLE = preferences.getFloat("OBS_ANG", FactoryDefaults::OBS_SWEEP_ANGLE);
        currentSettings.OBS_SWEEP_SPEED = preferences.getFloat("OBS_SPD", FactoryDefaults::OBS_SWEEP_SPEED);
        currentSettings.OBS_SWEEP_PAUSE = preferences.getInt("OBS_PAUSE", FactoryDefaults::OBS_SWEEP_PAUSE);
        currentSettings.OBS_CLEAR_THRESH = preferences.getFloat("OBS_CLR", FactoryDefaults::OBS_CLEAR_THRESH);
        currentSettings.OBS_HYSTERESIS = preferences.getFloat("OBS_HYST", FactoryDefaults::OBS_HYSTERESIS);

        // --- Obstacle Escape & Path Scan Sweep ---
        currentSettings.OBSTACLE_ESCAPE_BASE_SPEED = preferences.getFloat("OBS_ESC_SPD", FactoryDefaults::OBSTACLE_ESCAPE_BASE_SPEED);
        currentSettings.OBSTACLE_BACKUP_DURATION_MS = preferences.getInt("OBS_BKP_DUR", FactoryDefaults::OBSTACLE_BACKUP_DURATION_MS);
        currentSettings.OBSTACLE_SWEEP_ANGLE_DEG = preferences.getFloat("OBS_SWP_ANG", FactoryDefaults::OBSTACLE_SWEEP_ANGLE_DEG);
        currentSettings.OBSTACLE_SWEEP_TIMEOUT_MS = preferences.getInt("OBS_SWP_TO", FactoryDefaults::OBSTACLE_SWEEP_TIMEOUT_MS);
        currentSettings.OBSTACLE_ALIGN_TIMEOUT_MS = preferences.getInt("OBS_ALN_TO", FactoryDefaults::OBSTACLE_ALIGN_TIMEOUT_MS);
        currentSettings.OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG = preferences.getFloat("OBS_ALN_TOL", FactoryDefaults::OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG);
        currentSettings.OBSTACLE_ESCAPE_DURATION_MS = preferences.getInt("OBS_ESC_DUR", FactoryDefaults::OBSTACLE_ESCAPE_DURATION_MS);

        // --- Timers ---
        currentSettings.DIZZY_SPIN_PWM = preferences.getInt("DZZY_PWM", FactoryDefaults::DIZZY_SPIN_PWM);
        currentSettings.DIZZY_SPIN_TIME = preferences.getInt("DZZY_TIME", FactoryDefaults::DIZZY_SPIN_TIME);
        currentSettings.DIZZY_COOLDOWN = preferences.getInt("DZZY_COOL", FactoryDefaults::DIZZY_COOLDOWN);
        currentSettings.SLEEP_TIMEOUT_MS = preferences.getInt("SLP_TIME", FactoryDefaults::SLEEP_TIMEOUT_MS);
        currentSettings.SLEEP_WAKE_G = preferences.getFloat("SLP_WAKE", FactoryDefaults::SLEEP_WAKE_G);

        // --- Compass Lock Settings
        currentSettings.COMPASS_LOCK_ENTRY_SETTLE_MS = preferences.getInt("CMP_LCK_ENT_TO", FactoryDefaults::COMPASS_LOCK_ENTRY_SETTLE_MS);
        currentSettings.COMPASS_LOCK_EXIT_SETTLE_MS = preferences.getInt("CMP_LCK_EXT_TO", FactoryDefaults::COMPASS_LOCK_EXIT_SETTLE_MS);

        // --- Sensors ---
        currentSettings.IMU_GYRO_DEADBAND = preferences.getFloat("IMU_DEAD", FactoryDefaults::IMU_GYRO_DEADBAND);
        currentSettings.SONAR_MAX_DIST = preferences.getFloat("SNR_MAX", FactoryDefaults::SONAR_MAX_DIST);

        // --- IMU ORIENTATION ---
        currentSettings.IMU_INVERT_ROLL = preferences.getBool("IMU_INV_R", FactoryDefaults::IMU_INVERT_ROLL);
        currentSettings.IMU_INVERT_PITCH = preferences.getBool("IMU_INV_P", FactoryDefaults::IMU_INVERT_PITCH);
        currentSettings.IMU_INVERT_YAW = preferences.getBool("IMU_INV_Y", FactoryDefaults::IMU_INVERT_YAW);

        // --- Madgwick Filter ---
        currentSettings.MADGWICK_FILTER_BETA = preferences.getFloat("MDGW_FLTR_BETA", FactoryDefaults::MADGWICK_FILTER_BETA);

        // Autotune settings
        currentSettings.AUTOTUNE_START_DELAY_MS = preferences.getULong("AT_DELAY", FactoryDefaults::AUTOTUNE_START_DELAY_MS);
        currentSettings.AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS = preferences.getULong("AT_FAIL_TMOUT", FactoryDefaults::AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS);

        // --- PID: Unified Point Turn ---
        currentSettings.PID_POINT_P = preferences.getFloat("PID_PT_P", FactoryDefaults::PID_POINT_P);
        currentSettings.PID_POINT_I = preferences.getFloat("PID_PT_I", FactoryDefaults::PID_POINT_I);
        currentSettings.PID_POINT_D = preferences.getFloat("PID_PT_D", FactoryDefaults::PID_POINT_D);
        currentSettings.PID_POINT_LIM = preferences.getFloat("PID_PT_LIM", FactoryDefaults::PID_POINT_LIM);
        currentSettings.PID_POINT_ILIM = preferences.getFloat("PID_PT_ILM", FactoryDefaults::PID_POINT_ILIM);
        currentSettings.PID_POINT_DEAD = preferences.getFloat("PID_PT_DED", FactoryDefaults::PID_POINT_DEAD);

        // --- PID: Unified Arc Turn ---
        currentSettings.PID_ARC_P = preferences.getFloat("PID_ARC_P", FactoryDefaults::PID_ARC_P);
        currentSettings.PID_ARC_I = preferences.getFloat("PID_ARC_I", FactoryDefaults::PID_ARC_I);
        currentSettings.PID_ARC_D = preferences.getFloat("PID_ARC_D", FactoryDefaults::PID_ARC_D);
        currentSettings.PID_ARC_LIM = preferences.getFloat("PID_ARC_LIM", FactoryDefaults::PID_ARC_LIM);
        currentSettings.PID_ARC_ILIM = preferences.getFloat("PID_ARC_ILM", FactoryDefaults::PID_ARC_ILIM);
        currentSettings.PID_ARC_DEAD = preferences.getFloat("PID_ARC_DED", FactoryDefaults::PID_ARC_DEAD);

        /*
        // --- PID: Heading ---
        currentSettings.PID_HEADING_P = preferences.getFloat("PID_HDG_P", FactoryDefaults::PID_HEADING_P);
        currentSettings.PID_HEADING_I = preferences.getFloat("PID_HDG_I", FactoryDefaults::PID_HEADING_I);
        currentSettings.PID_HEADING_D = preferences.getFloat("PID_HDG_D", FactoryDefaults::PID_HEADING_D);
        currentSettings.PID_HEADING_LIM = preferences.getFloat("PID_HDG_LIM", FactoryDefaults::PID_HEADING_LIM);
        currentSettings.PID_HEADING_ILIM = preferences.getFloat("PID_HDG_ILM", FactoryDefaults::PID_HEADING_ILIM);
        currentSettings.PID_HEADING_DEAD = preferences.getFloat("PID_HDG_DED", FactoryDefaults::PID_HEADING_DEAD);

        // --- PID: Compass Lock ---
        currentSettings.PID_COMPASS_P = preferences.getFloat("PID_CMP_P", FactoryDefaults::PID_COMPASS_P);
        currentSettings.PID_COMPASS_I = preferences.getFloat("PID_CMP_I", FactoryDefaults::PID_COMPASS_I);
        currentSettings.PID_COMPASS_D = preferences.getFloat("PID_CMP_D", FactoryDefaults::PID_COMPASS_D);
        currentSettings.PID_COMPASS_LIM = preferences.getFloat("PID_CMP_LIM", FactoryDefaults::PID_COMPASS_LIM);
        currentSettings.PID_COMPASS_ILIM = preferences.getFloat("PID_CMP_ILM", FactoryDefaults::PID_COMPASS_ILIM);
        currentSettings.PID_COMPASS_DEAD = preferences.getFloat("PID_CMP_DED", FactoryDefaults::PID_COMPASS_DEAD);
        */

        // --- PID: Distance ---
        currentSettings.PID_DIST_P = preferences.getFloat("PID_DST_P", FactoryDefaults::PID_DIST_P);
        currentSettings.PID_DIST_I = preferences.getFloat("PID_DST_I", FactoryDefaults::PID_DIST_I);
        currentSettings.PID_DIST_D = preferences.getFloat("PID_DST_D", FactoryDefaults::PID_DIST_D);
        currentSettings.PID_DIST_LIM = preferences.getFloat("PID_DST_LIM", FactoryDefaults::PID_DIST_LIM);
        currentSettings.PID_DIST_ILIM = preferences.getFloat("PID_DST_ILM", FactoryDefaults::PID_DIST_ILIM);
        currentSettings.PID_DIST_DEAD = preferences.getFloat("PID_DST_DED", FactoryDefaults::PID_DIST_DEAD);

        /*
        // --- PID: Obstacle ---
        currentSettings.PID_OBSTACLE_P = preferences.getFloat("PID_OBS_P", FactoryDefaults::PID_OBSTACLE_P);
        currentSettings.PID_OBSTACLE_I = preferences.getFloat("PID_OBS_I", FactoryDefaults::PID_OBSTACLE_I);
        currentSettings.PID_OBSTACLE_D = preferences.getFloat("PID_OBS_D", FactoryDefaults::PID_OBSTACLE_D);
        currentSettings.PID_OBSTACLE_LIM = preferences.getFloat("PID_OBS_LIM", FactoryDefaults::PID_OBSTACLE_LIM);
        currentSettings.PID_OBSTACLE_ILIM = preferences.getFloat("PID_OBS_ILM", FactoryDefaults::PID_OBSTACLE_ILIM);
        currentSettings.PID_OBSTACLE_DEAD = preferences.getFloat("PID_OBS_DED", FactoryDefaults::PID_OBSTACLE_DEAD);
        */

        // --- Physics & Handing ---
        currentSettings.TILT_HANDLING_THRESHOLD = preferences.getFloat("TLT_HNDL_THRS", FactoryDefaults::TILT_HANDLING_THRESHOLD);
        currentSettings.GFORCE_LIFT_UP_THRESHOLD = preferences.getFloat("GF_LFT_UP_THRS", FactoryDefaults::GFORCE_LIFT_UP_THRESHOLD);
        currentSettings.GFORCE_LIFT_DOWN_THRESHOLD = preferences.getFloat("GF_LFT_DN_THRS", FactoryDefaults::GFORCE_LIFT_DOWN_THRESHOLD);
        currentSettings.LIFT_ENERGY_SPIKE_THRESHOLD = preferences.getFloat("LFT_E_SPK_THRS", FactoryDefaults::LIFT_ENERGY_SPIKE_THRESHOLD);
        currentSettings.UPRIGHT_ANGLE_TOLERANCE = preferences.getFloat("UPRT_ANGL_TOL", FactoryDefaults::UPRIGHT_ANGLE_TOLERANCE);
        currentSettings.PERFECTLY_STILL_ENERGY = preferences.getFloat("PERF_STILL_E", FactoryDefaults::PERFECTLY_STILL_ENERGY);
        currentSettings.STEADY_HOLD_ENERGY_MAX = preferences.getFloat("STD_HOLD_E_MAX", FactoryDefaults::STEADY_HOLD_ENERGY_MAX);

        // --- Dizzy & Energy---
        currentSettings.DIZZY_ENERGY_DEADBAND = preferences.getFloat("DZZY_E_DEADB", FactoryDefaults::DIZZY_ENERGY_DEADBAND);
        currentSettings.DIZZY_CHARGE_RATE = preferences.getFloat("DZZY_CHG_RATE", FactoryDefaults::DIZZY_CHARGE_RATE);
        currentSettings.DIZZY_DECAY_RATE = preferences.getFloat("DZZY_DEC_RATE", FactoryDefaults::DIZZY_DECAY_RATE);
        currentSettings.DIZZY_TRIGGER_THRESHOLD = preferences.getFloat("DZZY_TRIG_THRS", FactoryDefaults::DIZZY_TRIGGER_THRESHOLD);
        currentSettings.ENERGY_EMA_ALPHA = preferences.getFloat("E_EMA_ALPHA", FactoryDefaults::ENERGY_EMA_ALPHA);
        currentSettings.ENERGY_EMA_BETA = preferences.getFloat("E_EMA_BETA", FactoryDefaults::ENERGY_EMA_BETA);
        currentSettings.DIZZY_DURATION_MS = preferences.getInt("DZZY_DUR", FactoryDefaults::DIZZY_DURATION_MS);

        // --- Mood ---
        currentSettings.DISTANCE_HOLD_FRUSTRATION_LIMIT = preferences.getFloat("DST_HLD_FRST_L", FactoryDefaults::DISTANCE_HOLD_FRUSTRATION_LIMIT);
        currentSettings.FRUSTRATION_COOLDOWN_RATE = preferences.getFloat("FRST_COOL_RATE", FactoryDefaults::FRUSTRATION_COOLDOWN_RATE);
        currentSettings.FRUSTRATION_HEATUP_RATE = preferences.getFloat("FRST_HEAT_RATE", FactoryDefaults::FRUSTRATION_HEATUP_RATE);
        
        // --- System & Network ---
        currentSettings.BRAIN_ACTIVE = preferences.getBool("BRAIN_ACT", FactoryDefaults::BRAIN_ACTIVE);
        
        currentSettings.WIFI_SSID = preferences.getString("WIFI_SSID", FactoryDefaults::WIFI_SSID);
        currentSettings.WIFI_PASSWORD = preferences.getString("WIFI_PASS", FactoryDefaults::WIFI_PASSWORD);
        currentSettings.BT_NAME = preferences.getString("BT_NAME", FactoryDefaults::BT_NAME);

        currentSettings.WIFI_ACTIVE = preferences.getBool("WIFI_ACT", FactoryDefaults::WIFI_ACTIVE);
        currentSettings.BT_ACTIVE = preferences.getBool("BT_ACT", FactoryDefaults::BT_ACTIVE);
        
        currentSettings.SERIAL_DEBUG_MASTER = preferences.getBool("DBG_MASTER", FactoryDefaults::SERIAL_DEBUG_MASTER);
        currentSettings.SERIAL_DEBUG_IMU = preferences.getBool("DBG_IMU", FactoryDefaults::SERIAL_DEBUG_IMU);
        currentSettings.SERIAL_DEBUG_SONAR = preferences.getBool("DBG_SONAR", FactoryDefaults::SERIAL_DEBUG_SONAR);
        currentSettings.SERIAL_DEBUG_MOTOR_DRIVER = preferences.getBool("DBG_MOTOR", FactoryDefaults::SERIAL_DEBUG_MOTOR_DRIVER);
        
        currentSettings.DEBUG_ACTIVE = preferences.getUInt("DBG_ACTIVE", FactoryDefaults::DEBUG_ACTIVE);
        currentSettings.DEBUG_USB = preferences.getUInt("DBG_USB", FactoryDefaults::DEBUG_USB);
        currentSettings.DEBUG_WIFI = preferences.getUInt("DBG_WIFI", FactoryDefaults::DEBUG_WIFI);
        currentSettings.DEBUG_BLUETOOTH = preferences.getUInt("DBG_BLUETOOTH", FactoryDefaults::DEBUG_BLUETOOTH);
        currentSettings.ACTIVE_DEBUG_MODE = preferences.getUInt("ACT_DBG_MODE", FactoryDefaults::ACTIVE_DEBUG_MODE);
    }

    MasterSettings& get() { return currentSettings; }

    void save() {
        preferences.putULong("SERIAL_BAUD", currentSettings.SERIAL_BAUD_RATE);
        preferences.putFloat("CRUISE_SPD", currentSettings.CRUISING_SPEED);
        preferences.putFloat("OBS_TRIG_CM", currentSettings.OBSTACLE_TRIGGER_CM);
        preferences.putFloat("MAINT_DIST_CM", currentSettings.MAINTAIN_DISTANCE_CM);
        preferences.putInt("MOT_MIN", currentSettings.MOTOR_MIN_PWM);

        preferences.putFloat("OBS_ANG", currentSettings.OBS_SWEEP_ANGLE);
        preferences.putFloat("OBS_SPD", currentSettings.OBS_SWEEP_SPEED);
        preferences.putInt("OBS_PAUSE", currentSettings.OBS_SWEEP_PAUSE);
        preferences.putFloat("OBS_CLR", currentSettings.OBS_CLEAR_THRESH);
        preferences.putFloat("OBS_HYST", currentSettings.OBS_HYSTERESIS);

        preferences.putFloat("OBS_ESC_SPD", currentSettings.OBSTACLE_ESCAPE_BASE_SPEED);
        preferences.putInt("OBS_BKP_DUR", currentSettings.OBSTACLE_BACKUP_DURATION_MS);
        preferences.putFloat("OBS_SWP_ANG", currentSettings.OBSTACLE_SWEEP_ANGLE_DEG);
        preferences.putInt("OBS_SWP_TO", currentSettings.OBSTACLE_SWEEP_TIMEOUT_MS);
        preferences.putInt("OBS_ALN_TO", currentSettings.OBSTACLE_ALIGN_TIMEOUT_MS);
        preferences.putFloat("OBS_ALN_TOL", currentSettings.OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG);
        preferences.putInt("OBS_ESC_DUR", currentSettings.OBSTACLE_ESCAPE_DURATION_MS);

        preferences.putInt("DZZY_PWM", currentSettings.DIZZY_SPIN_PWM);
        preferences.putInt("DZZY_TIME", currentSettings.DIZZY_SPIN_TIME);
        preferences.putInt("DZZY_COOL", currentSettings.DIZZY_COOLDOWN);
        preferences.putInt("SLP_TIME", currentSettings.SLEEP_TIMEOUT_MS);
        preferences.putFloat("SLP_WAKE", currentSettings.SLEEP_WAKE_G);

        preferences.putInt("CMP_LCK_ENT_TO", currentSettings.COMPASS_LOCK_ENTRY_SETTLE_MS);
        preferences.putInt("CMP_LCK_EXT_TO", currentSettings.COMPASS_LOCK_EXIT_SETTLE_MS);

        preferences.putFloat("IMU_DEAD", currentSettings.IMU_GYRO_DEADBAND);
        preferences.putFloat("SNR_MAX", currentSettings.SONAR_MAX_DIST);

        preferences.putBool("IMU_INV_R", currentSettings.IMU_INVERT_ROLL);
        preferences.putBool("IMU_INV_P", currentSettings.IMU_INVERT_PITCH);
        preferences.putBool("IMU_INV_Y", currentSettings.IMU_INVERT_YAW);

        preferences.putFloat("MDGW_FLTR_BETA", currentSettings.MADGWICK_FILTER_BETA);

        preferences.putULong("AT_DELAY", currentSettings.AUTOTUNE_START_DELAY_MS);
        preferences.putULong("AT_FAIL_TMOUT", currentSettings.AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS);

        preferences.putFloat("PID_PT_P", currentSettings.PID_POINT_P);
        preferences.putFloat("PID_PT_I", currentSettings.PID_POINT_I);
        preferences.putFloat("PID_PT_D", currentSettings.PID_POINT_D);
        preferences.putFloat("PID_PT_LIM", currentSettings.PID_POINT_LIM);
        preferences.putFloat("PID_PT_ILM", currentSettings.PID_POINT_ILIM);
        preferences.putFloat("PID_PT_DED", currentSettings.PID_POINT_DEAD);

        preferences.putFloat("PID_ARC_P", currentSettings.PID_ARC_P);
        preferences.putFloat("PID_ARC_I", currentSettings.PID_ARC_I);
        preferences.putFloat("PID_ARC_D", currentSettings.PID_ARC_D);
        preferences.putFloat("PID_ARC_LIM", currentSettings.PID_ARC_LIM);
        preferences.putFloat("PID_ARC_ILM", currentSettings.PID_ARC_ILIM);
        preferences.putFloat("PID_ARC_DED", currentSettings.PID_ARC_DEAD);

        /*
        preferences.putFloat("PID_HDG_P", currentSettings.PID_HEADING_P);
        preferences.putFloat("PID_HDG_I", currentSettings.PID_HEADING_I);
        preferences.putFloat("PID_HDG_D", currentSettings.PID_HEADING_D);
        preferences.putFloat("PID_HDG_LIM", currentSettings.PID_HEADING_LIM);
        preferences.putFloat("PID_HDG_ILIM", currentSettings.PID_HEADING_ILIM);
        preferences.putFloat("PID_HDG_DED", currentSettings.PID_HEADING_DEAD);

        preferences.putFloat("PID_CMP_P", currentSettings.PID_COMPASS_P);
        preferences.putFloat("PID_CMP_I", currentSettings.PID_COMPASS_I);
        preferences.putFloat("PID_CMP_D", currentSettings.PID_COMPASS_D);
        preferences.putFloat("PID_CMP_LIM", currentSettings.PID_COMPASS_LIM);
        preferences.putFloat("PID_CMP_ILIM", currentSettings.PID_COMPASS_ILIM);
        preferences.putFloat("PID_CMP_DED", currentSettings.PID_COMPASS_DEAD);
        */

        preferences.putFloat("PID_DST_P", currentSettings.PID_DIST_P);
        preferences.putFloat("PID_DST_I", currentSettings.PID_DIST_I);
        preferences.putFloat("PID_DST_D", currentSettings.PID_DIST_D);
        preferences.putFloat("PID_DST_LIM", currentSettings.PID_DIST_LIM);
        preferences.putFloat("PID_DST_ILIM", currentSettings.PID_DIST_ILIM);
        preferences.putFloat("PID_DST_DED", currentSettings.PID_DIST_DEAD);

        /*
        preferences.putFloat("PID_OBS_P", currentSettings.PID_OBSTACLE_P);
        preferences.putFloat("PID_OBS_I", currentSettings.PID_OBSTACLE_I);
        preferences.putFloat("PID_OBS_D", currentSettings.PID_OBSTACLE_D);
        preferences.putFloat("PID_OBS_LIM", currentSettings.PID_OBSTACLE_LIM);
        preferences.putFloat("PID_OBS_ILIM", currentSettings.PID_OBSTACLE_ILIM);
        preferences.putFloat("PID_OBS_DED", currentSettings.PID_OBSTACLE_DEAD);
        */

        preferences.putFloat("TLT_HNDL_THRS", currentSettings.TILT_HANDLING_THRESHOLD);
        preferences.putFloat("GF_LFT_UP_THRS", currentSettings.GFORCE_LIFT_UP_THRESHOLD);
        preferences.putFloat("GF_LFT_DN_THRS", currentSettings.GFORCE_LIFT_DOWN_THRESHOLD);
        preferences.putFloat("LFT_E_SPK_THRS", currentSettings.LIFT_ENERGY_SPIKE_THRESHOLD);
        preferences.putFloat("UPRT_ANGL_TOL", currentSettings.UPRIGHT_ANGLE_TOLERANCE);
        preferences.putFloat("PERF_STILL_E", currentSettings.PERFECTLY_STILL_ENERGY);
        preferences.putFloat("STD_HOLD_E_MAX", currentSettings.STEADY_HOLD_ENERGY_MAX);
        preferences.putFloat("DZZY_E_DEADB", currentSettings.DIZZY_ENERGY_DEADBAND);
        preferences.putFloat("DZZY_CHG_RATE", currentSettings.DIZZY_CHARGE_RATE);
        preferences.putFloat("DZZY_DEC_RATE", currentSettings.DIZZY_DECAY_RATE);
        preferences.putFloat("DZZY_TRIG_THRS", currentSettings.DIZZY_TRIGGER_THRESHOLD);
        preferences.putFloat("E_EMA_ALPHA", currentSettings.ENERGY_EMA_ALPHA);
        preferences.putFloat("E_EMA_BETA", currentSettings.ENERGY_EMA_BETA);
        preferences.putFloat("DST_HLD_FRST_L", currentSettings.DISTANCE_HOLD_FRUSTRATION_LIMIT);
        preferences.putFloat("FRST_COOL_RATE", currentSettings.FRUSTRATION_COOLDOWN_RATE);
        preferences.putFloat("FRST_HEAT_RATE", currentSettings.FRUSTRATION_HEATUP_RATE);
        
        preferences.putBool("BRAIN_ACT", currentSettings.BRAIN_ACTIVE);

        if (currentSettings.WIFI_SSID == "") { if (preferences.isKey("WIFI_SSID")) preferences.remove("WIFI_SSID"); } 
        else { preferences.putString("WIFI_SSID", currentSettings.WIFI_SSID); }
        if (currentSettings.WIFI_PASSWORD == "") { if (preferences.isKey("WIFI_PASS")) preferences.remove("WIFI_PASS"); } 
        else { preferences.putString("WIFI_PASS", currentSettings.WIFI_PASSWORD); }
        if (currentSettings.BT_NAME == "") { if (preferences.isKey("BT_NAME")) preferences.remove("BT_NAME"); } 
        else { preferences.putString("BT_NAME", currentSettings.BT_NAME); }

        preferences.putBool("WIFI_ACT", currentSettings.WIFI_ACTIVE);
        preferences.putBool("BT_ACT", currentSettings.BT_ACTIVE);
        
        preferences.putBool("DBG_MASTER", currentSettings.SERIAL_DEBUG_MASTER);
        preferences.putBool("DBG_IMU", currentSettings.SERIAL_DEBUG_IMU);
        preferences.putBool("DBG_SONAR", currentSettings.SERIAL_DEBUG_SONAR);
        preferences.putBool("DBG_MOTOR", currentSettings.SERIAL_DEBUG_MOTOR_DRIVER);

        preferences.putUInt("DBG_ACTIVE", FactoryDefaults::DEBUG_ACTIVE);
        preferences.putUInt("DBG_USB", FactoryDefaults::DEBUG_USB);
        preferences.putUInt("DBG_WIFI", FactoryDefaults::DEBUG_WIFI);
        preferences.putUInt("DBG_BLUETOOTH", FactoryDefaults::DEBUG_BLUETOOTH);
        preferences.putUInt("ACT_DBG_MODE", currentSettings.ACTIVE_DEBUG_MODE);
    }

    // --- NEW: Granular Reset ---
    bool resetVariable(String varName) {
        varName.toUpperCase();
        /*if (varName == "CRUISING_SPEED") { currentSettings.CRUISING_SPEED = FactoryDefaults::CRUISING_SPEED; }
        else if (varName == "OBSTACLE_TRIGGER_CM") { currentSettings.OBSTACLE_TRIGGER_CM = FactoryDefaults::OBSTACLE_TRIGGER_CM; }
        else if (varName == "MAINTAIN_DIST_CM") { currentSettings.MAINTAIN_DIST_CM = FactoryDefaults::MAINTAIN_DISTANCE_CM; }

        else if (varName == "BRAIN_ACTIVE") { currentSettings.BRAIN_ACTIVE = FactoryDefaults::BRAIN_ACTIVE; }

        else if (varName == "WIFI_SSID") { currentSettings.WIFI_SSID = FactoryDefaults::WIFI_SSID; }
        else if (varName == "WIFI_PASSWORD") { currentSettings.WIFI_PASSWORD = FactoryDefaults::WIFI_PASSWORD; }
        else if (varName == "WIFI_ACTIVE") { currentSettings.WIFI_ACTIVE = FactoryDefaults::WIFI_ACTIVE; }
        
        else if (varName == "BT_NAME") { currentSettings.BT_NAME = FactoryDefaults::BT_NAME; }
        else if (varName == "BT_ACTIVE") { currentSettings.BT_ACTIVE = FactoryDefaults::BT_ACTIVE; }

        else if (varName == "SERIAL_DEBUG_MASTER") { currentSettings.SERIAL_DEBUG_MASTER = FactoryDefaults::SERIAL_DEBUG_MASTER; }
        else if (varName == "ACTIVE_DEBUG_MODE") { currentSettings.ACTIVE_DEBUG_MODE = FactoryDefaults::ACTIVE_DEBUG_MODE; }*/

        if (varName == "SERIAL_BAUD_RATE") { currentSettings.SERIAL_BAUD_RATE = FactoryDefaults::SERIAL_BAUD_RATE; }
        else if (varName == "CRUISING_SPEED") { currentSettings.CRUISING_SPEED = FactoryDefaults::CRUISING_SPEED; }
        else if (varName == "OBSTACLE_TRIGGER_CM") { currentSettings.OBSTACLE_TRIGGER_CM = FactoryDefaults::OBSTACLE_TRIGGER_CM; }
        else if (varName == "MAINTAIN_DISTANCE_CM") { currentSettings.MAINTAIN_DISTANCE_CM = FactoryDefaults::MAINTAIN_DISTANCE_CM; }
        else if (varName == "MOTOR_MIN_PWM") { currentSettings.MOTOR_MIN_PWM = FactoryDefaults::MOTOR_MIN_PWM; }

        else if (varName == "OBS_SWEEP_ANGLE") { currentSettings.OBS_SWEEP_ANGLE = FactoryDefaults::OBS_SWEEP_ANGLE; }
        else if (varName == "OBS_SWEEP_SPEED") { currentSettings.OBS_SWEEP_SPEED = FactoryDefaults::OBS_SWEEP_SPEED; }
        else if (varName == "OBS_SWEEP_PAUSE") { currentSettings.OBS_SWEEP_PAUSE = FactoryDefaults::OBS_SWEEP_PAUSE; }
        else if (varName == "OBS_CLEAR_THRESH") { currentSettings.OBS_CLEAR_THRESH = FactoryDefaults::OBS_CLEAR_THRESH; }
        else if (varName == "OBS_HYSTERESIS") { currentSettings.OBS_HYSTERESIS = FactoryDefaults::OBS_HYSTERESIS; }

        else if (varName == "OBSTACLE_ESCAPE_BASE_SPEED") { currentSettings.OBSTACLE_ESCAPE_BASE_SPEED = FactoryDefaults::OBSTACLE_ESCAPE_BASE_SPEED; }
        else if (varName == "OBSTACLE_BACKUP_DURATION_MS") { currentSettings.OBSTACLE_BACKUP_DURATION_MS = FactoryDefaults::OBSTACLE_BACKUP_DURATION_MS; }
        else if (varName == "OBSTACLE_SWEEP_ANGLE_DEG") { currentSettings.OBSTACLE_SWEEP_ANGLE_DEG = FactoryDefaults::OBSTACLE_SWEEP_ANGLE_DEG; }
        else if (varName == "OBSTACLE_SWEEP_TIMEOUT_MS") { currentSettings.OBSTACLE_SWEEP_TIMEOUT_MS = FactoryDefaults::OBSTACLE_SWEEP_TIMEOUT_MS; }
        else if (varName == "OBSTACLE_ALIGN_TIMEOUT_MS") { currentSettings.OBSTACLE_ALIGN_TIMEOUT_MS = FactoryDefaults::OBSTACLE_ALIGN_TIMEOUT_MS; }
        else if (varName == "OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG") { currentSettings.OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG = FactoryDefaults::OBSTACLE_ALIGN_SUCCESS_TOLERANCE_DEG; }
        else if (varName == "OBSTACLE_ESCAPE_DURATION_MS") { currentSettings.OBSTACLE_ESCAPE_DURATION_MS = FactoryDefaults::OBSTACLE_ESCAPE_DURATION_MS; }

        else if (varName == "DIZZY_SPIN_PWM") { currentSettings.DIZZY_SPIN_PWM = FactoryDefaults::DIZZY_SPIN_PWM; }
        else if (varName == "DIZZY_SPIN_TIME") { currentSettings.DIZZY_SPIN_TIME = FactoryDefaults::DIZZY_SPIN_TIME; }
        else if (varName == "DIZZY_COOLDOWN") { currentSettings.DIZZY_COOLDOWN = FactoryDefaults::DIZZY_COOLDOWN; }
        else if (varName == "SLEEP_TIMEOUT_MS") { currentSettings.SLEEP_TIMEOUT_MS = FactoryDefaults::SLEEP_TIMEOUT_MS; }
        else if (varName == "SLEEP_WAKE_G") { currentSettings.SLEEP_WAKE_G = FactoryDefaults::SLEEP_WAKE_G; }

        else if (varName == "COMPASS_LOCK_ENTRY_SETTLE_MS") { currentSettings.COMPASS_LOCK_ENTRY_SETTLE_MS = FactoryDefaults::COMPASS_LOCK_ENTRY_SETTLE_MS; }
        else if (varName == "COMPASS_LOCK_EXIT_SETTLE_MS") { currentSettings.COMPASS_LOCK_EXIT_SETTLE_MS = FactoryDefaults::COMPASS_LOCK_EXIT_SETTLE_MS; }

        else if (varName == "IMU_GYRO_DEADBAND") { currentSettings.IMU_GYRO_DEADBAND = FactoryDefaults::IMU_GYRO_DEADBAND; }
        else if (varName == "SONAR_MAX_DIST") { currentSettings.SONAR_MAX_DIST = FactoryDefaults::SONAR_MAX_DIST; }

        else if (varName == "IMU_INVERT_ROLL") { currentSettings.IMU_INVERT_ROLL = FactoryDefaults::IMU_INVERT_ROLL; }
        else if (varName == "IMU_INVERT_PITCH") { currentSettings.IMU_INVERT_PITCH = FactoryDefaults::IMU_INVERT_PITCH; }
        else if (varName == "IMU_INVERT_YAW") { currentSettings.IMU_INVERT_YAW = FactoryDefaults::IMU_INVERT_YAW; }

        else if (varName == "MADGWICK_FILTER_BETA") { currentSettings.MADGWICK_FILTER_BETA = FactoryDefaults::MADGWICK_FILTER_BETA; }

        else if (varName == "AUTOTUNE_START_DELAY_MS") { currentSettings.AUTOTUNE_START_DELAY_MS = FactoryDefaults::AUTOTUNE_START_DELAY_MS; }
        else if (varName == "AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS") { currentSettings.AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS = FactoryDefaults::AUTOTUNE_UNSUCCESSFUL_TIMEOUT_MS; }

        else if (varName == "PID_POINT_P") { currentSettings.PID_POINT_P = FactoryDefaults::PID_POINT_P; }
        else if (varName == "PID_POINT_I") { currentSettings.PID_POINT_I = FactoryDefaults::PID_POINT_I; }
        else if (varName == "PID_POINT_D") { currentSettings.PID_POINT_D = FactoryDefaults::PID_POINT_D; }
        else if (varName == "PID_POINT_LIM") { currentSettings.PID_POINT_LIM = FactoryDefaults::PID_POINT_LIM; }
        else if (varName == "PID_POINT_ILIM") { currentSettings.PID_POINT_ILIM = FactoryDefaults::PID_POINT_ILIM; }
        else if (varName == "PID_POINT_DEAD") { currentSettings.PID_POINT_DEAD = FactoryDefaults::PID_POINT_DEAD; }

        else if (varName == "PID_ARC_P") { currentSettings.PID_ARC_P = FactoryDefaults::PID_ARC_P; }
        else if (varName == "PID_ARC_I") { currentSettings.PID_ARC_I = FactoryDefaults::PID_ARC_I; }
        else if (varName == "PID_ARC_D") { currentSettings.PID_ARC_D = FactoryDefaults::PID_ARC_D; }
        else if (varName == "PID_ARC_LIM") { currentSettings.PID_ARC_LIM = FactoryDefaults::PID_ARC_LIM; }
        else if (varName == "PID_ARC_ILIM") { currentSettings.PID_ARC_ILIM = FactoryDefaults::PID_ARC_ILIM; }
        else if (varName == "PID_ARC_DEAD") { currentSettings.PID_ARC_DEAD = FactoryDefaults::PID_ARC_DEAD; }

        /*
        else if (varName == "PID_HEADING_P") { currentSettings.PID_HEADING_P = FactoryDefaults::PID_HEADING_P; }
        else if (varName == "PID_HEADING_I") { currentSettings.PID_HEADING_I = FactoryDefaults::PID_HEADING_I; }
        else if (varName == "PID_HEADING_D") { currentSettings.PID_HEADING_D = FactoryDefaults::PID_HEADING_D; }
        else if (varName == "PID_HEADING_LIM") { currentSettings.PID_HEADING_LIM = FactoryDefaults::PID_HEADING_LIM; }
        else if (varName == "PID_HEADING_ILIM") { currentSettings.PID_HEADING_ILIM = FactoryDefaults::PID_HEADING_ILIM; }
        else if (varName == "PID_HEADING_DEAD") { currentSettings.PID_HEADING_DEAD = FactoryDefaults::PID_HEADING_DEAD; }

        else if (varName == "PID_COMPASS_P") { currentSettings.PID_COMPASS_P = FactoryDefaults::PID_COMPASS_P; }
        else if (varName == "PID_COMPASS_I") { currentSettings.PID_COMPASS_I = FactoryDefaults::PID_COMPASS_I; }
        else if (varName == "PID_COMPASS_D") { currentSettings.PID_COMPASS_D = FactoryDefaults::PID_COMPASS_D; }
        else if (varName == "PID_COMPASS_LIM") { currentSettings.PID_COMPASS_LIM = FactoryDefaults::PID_COMPASS_LIM; }
        else if (varName == "PID_COMPASS_ILIM") { currentSettings.PID_COMPASS_ILIM = FactoryDefaults::PID_COMPASS_ILIM; }
        else if (varName == "PID_COMPASS_DEAD") { currentSettings.PID_COMPASS_DEAD = FactoryDefaults::PID_COMPASS_DEAD; }
        */

        else if (varName == "PID_DIST_P") { currentSettings.PID_DIST_P = FactoryDefaults::PID_DIST_P; }
        else if (varName == "PID_DIST_I") { currentSettings.PID_DIST_I = FactoryDefaults::PID_DIST_I; }
        else if (varName == "PID_DIST_D") { currentSettings.PID_DIST_D = FactoryDefaults::PID_DIST_D; }
        else if (varName == "PID_DIST_LIM") { currentSettings.PID_DIST_LIM = FactoryDefaults::PID_DIST_LIM; }
        else if (varName == "PID_DIST_ILIM") { currentSettings.PID_DIST_ILIM = FactoryDefaults::PID_DIST_ILIM; }
        else if (varName == "PID_DIST_DEAD") { currentSettings.PID_DIST_DEAD = FactoryDefaults::PID_DIST_DEAD; }

        /*
        else if (varName == "PID_OBSTACLE_P") { currentSettings.PID_OBSTACLE_P = FactoryDefaults::PID_OBSTACLE_P; }
        else if (varName == "PID_OBSTACLE_I") { currentSettings.PID_OBSTACLE_I = FactoryDefaults::PID_OBSTACLE_I; }
        else if (varName == "PID_OBSTACLE_D") { currentSettings.PID_OBSTACLE_D = FactoryDefaults::PID_OBSTACLE_D; }
        else if (varName == "PID_OBSTACLE_LIM") { currentSettings.PID_OBSTACLE_LIM = FactoryDefaults::PID_OBSTACLE_LIM; }
        else if (varName == "PID_OBSTACLE_ILIM") { currentSettings.PID_OBSTACLE_ILIM = FactoryDefaults::PID_OBSTACLE_ILIM; }
        else if (varName == "PID_OBSTACLE_DEAD") { currentSettings.PID_OBSTACLE_DEAD = FactoryDefaults::PID_OBSTACLE_DEAD; }
        */

        else if (varName == "TILT_HANDLING_THRESHOLD") { currentSettings.TILT_HANDLING_THRESHOLD = FactoryDefaults::TILT_HANDLING_THRESHOLD; }
        else if (varName == "GFORCE_LIFT_UP_THRESHOLD") { currentSettings.GFORCE_LIFT_UP_THRESHOLD = FactoryDefaults::GFORCE_LIFT_UP_THRESHOLD; }
        else if (varName == "GFORCE_LIFT_DOWN_THRESHOLD") { currentSettings.GFORCE_LIFT_DOWN_THRESHOLD = FactoryDefaults::GFORCE_LIFT_DOWN_THRESHOLD; }
        else if (varName == "LIFT_ENERGY_SPIKE_THRESHOLD") { currentSettings.LIFT_ENERGY_SPIKE_THRESHOLD = FactoryDefaults::LIFT_ENERGY_SPIKE_THRESHOLD; }
        else if (varName == "UPRIGHT_ANGLE_TOLERANCE") { currentSettings.UPRIGHT_ANGLE_TOLERANCE = FactoryDefaults::UPRIGHT_ANGLE_TOLERANCE; }
        else if (varName == "PERFECTLY_STILL_ENERGY") { currentSettings.PERFECTLY_STILL_ENERGY = FactoryDefaults::PERFECTLY_STILL_ENERGY; }
        else if (varName == "STEADY_HOLD_ENERGY_MAX") { currentSettings.STEADY_HOLD_ENERGY_MAX = FactoryDefaults::STEADY_HOLD_ENERGY_MAX; }
        else if (varName == "DIZZY_ENERGY_DEADBAND") { currentSettings.DIZZY_ENERGY_DEADBAND = FactoryDefaults::DIZZY_ENERGY_DEADBAND; }
        else if (varName == "DIZZY_CHARGE_RATE") { currentSettings.DIZZY_CHARGE_RATE = FactoryDefaults::DIZZY_CHARGE_RATE; }
        else if (varName == "DIZZY_DECAY_RATE") { currentSettings.DIZZY_DECAY_RATE = FactoryDefaults::DIZZY_DECAY_RATE; }
        else if (varName == "DIZZY_TRIGGER_THRESHOLD") { currentSettings.DIZZY_TRIGGER_THRESHOLD = FactoryDefaults::DIZZY_TRIGGER_THRESHOLD; }
        else if (varName == "ENERGY_EMA_ALPHA") { currentSettings.ENERGY_EMA_ALPHA = FactoryDefaults::ENERGY_EMA_ALPHA; }
        else if (varName == "ENERGY_EMA_BETA") { currentSettings.ENERGY_EMA_BETA = FactoryDefaults::ENERGY_EMA_BETA; }
        else if (varName == "DISTANCE_HOLD_FRUSTRATION_LIMIT") { currentSettings.DISTANCE_HOLD_FRUSTRATION_LIMIT = FactoryDefaults::DISTANCE_HOLD_FRUSTRATION_LIMIT; }
        else if (varName == "FRUSTRATION_COOLDOWN_RATE") { currentSettings.FRUSTRATION_COOLDOWN_RATE = FactoryDefaults::FRUSTRATION_COOLDOWN_RATE; }
        else if (varName == "FRUSTRATION_HEATUP_RATE") { currentSettings.FRUSTRATION_HEATUP_RATE = FactoryDefaults::FRUSTRATION_HEATUP_RATE; }

        else if (varName == "BRAIN_ACTIVE") { currentSettings.BRAIN_ACTIVE = FactoryDefaults::BRAIN_ACTIVE; }

        else if (varName == "WIFI_SSID") { currentSettings.WIFI_SSID = FactoryDefaults::WIFI_SSID; }
        else if (varName == "WIFI_PASSWORD") { currentSettings.WIFI_PASSWORD = FactoryDefaults::WIFI_PASSWORD; }
        else if (varName == "WIFI_ACTIVE") { currentSettings.WIFI_ACTIVE = FactoryDefaults::WIFI_ACTIVE; }
        else if (varName == "BT_NAME") { currentSettings.BT_NAME = FactoryDefaults::BT_NAME; }
        else if (varName == "BT_ACTIVE") { currentSettings.BT_ACTIVE = FactoryDefaults::BT_ACTIVE; }

        else if (varName == "SERIAL_DEBUG_MASTER") { currentSettings.SERIAL_DEBUG_MASTER = FactoryDefaults::SERIAL_DEBUG_MASTER; }
        else if (varName == "SERIAL_DEBUG_IMU") { currentSettings.SERIAL_DEBUG_IMU = FactoryDefaults::SERIAL_DEBUG_IMU; }
        else if (varName == "SERIAL_DEBUG_SONAR") { currentSettings.SERIAL_DEBUG_SONAR = FactoryDefaults::SERIAL_DEBUG_SONAR; }
        else if (varName == "SERIAL_DEBUG_MOTOR_DRIVER") { currentSettings.SERIAL_DEBUG_MOTOR_DRIVER = FactoryDefaults::SERIAL_DEBUG_MOTOR_DRIVER; }

        else if (varName == "DEBUG_ACTIVE") { currentSettings.DEBUG_ACTIVE = FactoryDefaults::DEBUG_ACTIVE; }
        else if (varName == "DEBUG_USB") { currentSettings.DEBUG_USB = FactoryDefaults::DEBUG_USB; }
        else if (varName == "DEBUG_WIFI") { currentSettings.DEBUG_WIFI = FactoryDefaults::DEBUG_WIFI; }
        else if (varName == "DEBUG_BLUETOOTH") { currentSettings.DEBUG_BLUETOOTH = FactoryDefaults::DEBUG_BLUETOOTH; }
        else if (varName == "ACTIVE_DEBUG_MODE") { currentSettings.ACTIVE_DEBUG_MODE = FactoryDefaults::ACTIVE_DEBUG_MODE; }

        else { return false; } // Variable not found
        save();
        return true;
    }

    void factoryReset() {
        preferences.clear(); 
        currentSettings = MasterSettings(); // Struct re-initializes to defaults
        save(); 
    }

    //MasterSettings& getSettings() { return currentSettings; }
};

#define Config ConfigurationManager::getInstance().get()
#define ConfigSys ConfigurationManager::getInstance()