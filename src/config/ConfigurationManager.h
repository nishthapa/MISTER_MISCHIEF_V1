#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "config/FactoryDefaults.h" // <-- Include our new master defaults

struct MasterSettings {
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

    // --- Timers ---
    int DIZZY_SPIN_PWM = FactoryDefaults::DIZZY_SPIN_PWM;
    int DIZZY_SPIN_TIME = FactoryDefaults::DIZZY_SPIN_TIME;
    int DIZZY_COOLDOWN = FactoryDefaults::DIZZY_COOLDOWN;
    int SLEEP_TIMEOUT_MS = FactoryDefaults::SLEEP_TIMEOUT_MS;
    float SLEEP_WAKE_G = FactoryDefaults::SLEEP_WAKE_G;

    // --- Sensors ---
    float IMU_MADGWICK_BETA = FactoryDefaults::IMU_MADGWICK_BETA;
    float IMU_GYRO_DEADBAND = FactoryDefaults::IMU_GYRO_DEADBAND;
    float SONAR_MAX_DIST = FactoryDefaults::SONAR_MAX_DIST;

    // --- PID: Heading ---
    float PID_HEADING_P = FactoryDefaults::PID_HEADING_P;
    float PID_HEADING_I = FactoryDefaults::PID_HEADING_I;
    float PID_HEADING_D = FactoryDefaults::PID_HEADING_D;
    float PID_HEADING_LIM = FactoryDefaults::PID_HEADING_LIM;
    float PID_HEADING_ILIM = FactoryDefaults::PID_HEADING_ILIM;
    float PID_HEADING_DEAD = FactoryDefaults::PID_HEADING_DEAD;

    // --- PID: Compass ---
    float PID_COMPASS_P = FactoryDefaults::PID_COMPASS_P;
    float PID_COMPASS_I = FactoryDefaults::PID_COMPASS_I;
    float PID_COMPASS_D = FactoryDefaults::PID_COMPASS_D;
    float PID_COMPASS_LIM = FactoryDefaults::PID_COMPASS_LIM;
    float PID_COMPASS_ILIM = FactoryDefaults::PID_COMPASS_ILIM;
    float PID_COMPASS_DEAD = FactoryDefaults::PID_COMPASS_DEAD;

    // --- PID: Distance ---
    float PID_DIST_P = FactoryDefaults::PID_DIST_P;
    float PID_DIST_I = FactoryDefaults::PID_DIST_I;
    float PID_DIST_D = FactoryDefaults::PID_DIST_D;
    float PID_DIST_LIM = FactoryDefaults::PID_DIST_LIM;
    float PID_DIST_ILIM = FactoryDefaults::PID_DIST_ILIM;
    float PID_DIST_DEAD = FactoryDefaults::PID_DIST_DEAD;

    // --- PID: Obstacle ---
    float PID_OBSTACLE_P = FactoryDefaults::PID_OBSTACLE_P;
    float PID_OBSTACLE_I = FactoryDefaults::PID_OBSTACLE_I;
    float PID_OBSTACLE_D = FactoryDefaults::PID_OBSTACLE_D;
    float PID_OBSTACLE_LIM = FactoryDefaults::PID_OBSTACLE_LIM;
    float PID_OBSTACLE_ILIM = FactoryDefaults::PID_OBSTACLE_ILIM;
    float PID_OBSTACLE_DEAD = FactoryDefaults::PID_OBSTACLE_DEAD;

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

    void init() {
        preferences.begin("mischief", false);
        currentSettings.CRUISING_SPEED = preferences.getFloat("CRUISE_SPD", FactoryDefaults::CRUISING_SPEED);
        currentSettings.OBSTACLE_TRIGGER_CM = preferences.getFloat("OBS_TRIG_CM", FactoryDefaults::OBSTACLE_TRIGGER_CM);
        currentSettings.MAINTAIN_DISTANCE_CM = preferences.getFloat("MAINT_DIST_CM", FactoryDefaults::MAINTAIN_DISTANCE_CM);
        currentSettings.MOTOR_MIN_PWM = preferences.getInt("MOT_MIN", FactoryDefaults::MOTOR_MIN_PWM);

        currentSettings.OBS_SWEEP_ANGLE = preferences.getFloat("OBS_ANG", FactoryDefaults::OBS_SWEEP_ANGLE);
        currentSettings.OBS_SWEEP_SPEED = preferences.getFloat("OBS_SPD", FactoryDefaults::OBS_SWEEP_SPEED);
        currentSettings.OBS_SWEEP_PAUSE = preferences.getInt("OBS_PAUSE", FactoryDefaults::OBS_SWEEP_PAUSE);
        currentSettings.OBS_CLEAR_THRESH = preferences.getFloat("OBS_CLR", FactoryDefaults::OBS_CLEAR_THRESH);
        currentSettings.OBS_HYSTERESIS = preferences.getFloat("OBS_HYST", FactoryDefaults::OBS_HYSTERESIS);

        currentSettings.DIZZY_SPIN_PWM = preferences.getInt("DIZ_PWM", FactoryDefaults::DIZZY_SPIN_PWM);
        currentSettings.DIZZY_SPIN_TIME = preferences.getInt("DIZ_TIME", FactoryDefaults::DIZZY_SPIN_TIME);
        currentSettings.DIZZY_COOLDOWN = preferences.getInt("DIZ_COOL", FactoryDefaults::DIZZY_COOLDOWN);
        currentSettings.SLEEP_TIMEOUT_MS = preferences.getInt("SLP_TIME", FactoryDefaults::SLEEP_TIMEOUT_MS);
        currentSettings.SLEEP_WAKE_G = preferences.getFloat("SLP_WAKE", FactoryDefaults::SLEEP_WAKE_G);

        currentSettings.IMU_MADGWICK_BETA = preferences.getFloat("IMU_BETA", FactoryDefaults::IMU_MADGWICK_BETA);
        currentSettings.IMU_GYRO_DEADBAND = preferences.getFloat("IMU_DEAD", FactoryDefaults::IMU_GYRO_DEADBAND);
        currentSettings.SONAR_MAX_DIST = preferences.getFloat("SNR_MAX", FactoryDefaults::SONAR_MAX_DIST);

        currentSettings.PID_HEADING_P = preferences.getFloat("P_HDG_P", FactoryDefaults::PID_HEADING_P);
        currentSettings.PID_HEADING_I = preferences.getFloat("P_HDG_I", FactoryDefaults::PID_HEADING_I);
        currentSettings.PID_HEADING_D = preferences.getFloat("P_HDG_D", FactoryDefaults::PID_HEADING_D);
        currentSettings.PID_HEADING_LIM = preferences.getFloat("P_HDG_LIM", FactoryDefaults::PID_HEADING_LIM);
        currentSettings.PID_HEADING_ILIM = preferences.getFloat("P_HDG_ILM", FactoryDefaults::PID_HEADING_ILIM);
        currentSettings.PID_HEADING_DEAD = preferences.getFloat("P_HDG_DED", FactoryDefaults::PID_HEADING_DEAD);

        currentSettings.PID_COMPASS_P = preferences.getFloat("P_CMP_P", FactoryDefaults::PID_COMPASS_P);
        currentSettings.PID_COMPASS_I = preferences.getFloat("P_CMP_I", FactoryDefaults::PID_COMPASS_I);
        currentSettings.PID_COMPASS_D = preferences.getFloat("P_CMP_D", FactoryDefaults::PID_COMPASS_D);
        currentSettings.PID_COMPASS_LIM = preferences.getFloat("P_CMP_LIM", FactoryDefaults::PID_COMPASS_LIM);
        currentSettings.PID_COMPASS_ILIM = preferences.getFloat("P_CMP_ILM", FactoryDefaults::PID_COMPASS_ILIM);
        currentSettings.PID_COMPASS_DEAD = preferences.getFloat("P_CMP_DED", FactoryDefaults::PID_COMPASS_DEAD);

        currentSettings.PID_DIST_P = preferences.getFloat("P_DST_P", FactoryDefaults::PID_DIST_P);
        currentSettings.PID_DIST_I = preferences.getFloat("P_DST_I", FactoryDefaults::PID_DIST_I);
        currentSettings.PID_DIST_D = preferences.getFloat("P_DST_D", FactoryDefaults::PID_DIST_D);
        currentSettings.PID_DIST_LIM = preferences.getFloat("P_DST_LIM", FactoryDefaults::PID_DIST_LIM);
        currentSettings.PID_DIST_ILIM = preferences.getFloat("P_DST_ILM", FactoryDefaults::PID_DIST_ILIM);
        currentSettings.PID_DIST_DEAD = preferences.getFloat("P_DST_DED", FactoryDefaults::PID_DIST_DEAD);

        currentSettings.PID_OBSTACLE_P = preferences.getFloat("P_OBS_P", FactoryDefaults::PID_OBSTACLE_P);
        currentSettings.PID_OBSTACLE_I = preferences.getFloat("P_OBS_I", FactoryDefaults::PID_OBSTACLE_I);
        currentSettings.PID_OBSTACLE_D = preferences.getFloat("P_OBS_D", FactoryDefaults::PID_OBSTACLE_D);
        currentSettings.PID_OBSTACLE_LIM = preferences.getFloat("P_OBS_LIM", FactoryDefaults::PID_OBSTACLE_LIM);
        currentSettings.PID_OBSTACLE_ILIM = preferences.getFloat("P_OBS_ILM", FactoryDefaults::PID_OBSTACLE_ILIM);
        currentSettings.PID_OBSTACLE_DEAD = preferences.getFloat("P_OBS_DED", FactoryDefaults::PID_OBSTACLE_DEAD);

        currentSettings.BRAIN_ACTIVE = preferences.getBool("BRAIN_ACT", FactoryDefaults::BRAIN_ACTIVE);
        
        currentSettings.WIFI_SSID = preferences.getString("WIFI_SSID", FactoryDefaults::WIFI_SSID);
        currentSettings.WIFI_PASSWORD = preferences.getString("WIFI_PASS", FactoryDefaults::WIFI_PASSWORD);
        currentSettings.WIFI_ACTIVE = preferences.getBool("WIFI_ACT", FactoryDefaults::WIFI_ACTIVE);
        currentSettings.BT_NAME = preferences.getString("BT_NAME", FactoryDefaults::BT_NAME);
        currentSettings.BT_ACTIVE = preferences.getBool("BT_ACT", FactoryDefaults::BT_ACTIVE);
        
        currentSettings.SERIAL_DEBUG_MASTER = preferences.getBool("DBG_MASTER", FactoryDefaults::SERIAL_DEBUG_MASTER);
        currentSettings.SERIAL_DEBUG_IMU = preferences.getBool("DBG_IMU", FactoryDefaults::SERIAL_DEBUG_IMU);
        currentSettings.SERIAL_DEBUG_SONAR = preferences.getBool("DBG_SONAR", FactoryDefaults::SERIAL_DEBUG_SONAR);
        currentSettings.SERIAL_DEBUG_MOTOR_DRIVER = preferences.getBool("DBG_MOTOR", FactoryDefaults::SERIAL_DEBUG_MOTOR_DRIVER);
    }

    MasterSettings& get() { return currentSettings; }

    void save() {
        preferences.putFloat("CRUISE_SPD", currentSettings.CRUISING_SPEED);
        preferences.putFloat("OBS_TRIG_CM", currentSettings.OBSTACLE_TRIGGER_CM);
        preferences.putFloat("MAINT_DIST_CM", currentSettings.MAINTAIN_DISTANCE_CM);
        preferences.putInt("MOT_MIN", currentSettings.MOTOR_MIN_PWM);

        preferences.putFloat("OBS_ANG", currentSettings.OBS_SWEEP_ANGLE);
        preferences.putFloat("OBS_SPD", currentSettings.OBS_SWEEP_SPEED);
        preferences.putInt("OBS_PAUSE", currentSettings.OBS_SWEEP_PAUSE);
        preferences.putFloat("OBS_CLR", currentSettings.OBS_CLEAR_THRESH);
        preferences.putFloat("OBS_HYST", currentSettings.OBS_HYSTERESIS);

        preferences.putInt("DIZ_PWM", currentSettings.DIZZY_SPIN_PWM);
        preferences.putInt("DIZ_TIME", currentSettings.DIZZY_SPIN_TIME);
        preferences.putInt("DIZ_COOL", currentSettings.DIZZY_COOLDOWN);
        preferences.putInt("SLP_TIME", currentSettings.SLEEP_TIMEOUT_MS);
        preferences.putFloat("SLP_WAKE", currentSettings.SLEEP_WAKE_G);

        preferences.putFloat("IMU_BETA", currentSettings.IMU_MADGWICK_BETA);
        preferences.putFloat("IMU_DEAD", currentSettings.IMU_GYRO_DEADBAND);
        preferences.putFloat("SNR_MAX", currentSettings.SONAR_MAX_DIST);

        preferences.putFloat("P_HDG_P", currentSettings.PID_HEADING_P);
        preferences.putFloat("P_HDG_I", currentSettings.PID_HEADING_I);
        preferences.putFloat("P_HDG_D", currentSettings.PID_HEADING_D);
        preferences.putFloat("P_HDG_LIM", currentSettings.PID_HEADING_LIM);
        preferences.putFloat("P_HDG_ILM", currentSettings.PID_HEADING_ILIM);
        preferences.putFloat("P_HDG_DED", currentSettings.PID_HEADING_DEAD);

        preferences.putFloat("P_CMP_P", currentSettings.PID_COMPASS_P);
        preferences.putFloat("P_CMP_I", currentSettings.PID_COMPASS_I);
        preferences.putFloat("P_CMP_D", currentSettings.PID_COMPASS_D);
        preferences.putFloat("P_CMP_LIM", currentSettings.PID_COMPASS_LIM);
        preferences.putFloat("P_CMP_ILM", currentSettings.PID_COMPASS_ILIM);
        preferences.putFloat("P_CMP_DED", currentSettings.PID_COMPASS_DEAD);

        preferences.putFloat("P_DST_P", currentSettings.PID_DIST_P);
        preferences.putFloat("P_DST_I", currentSettings.PID_DIST_I);
        preferences.putFloat("P_DST_D", currentSettings.PID_DIST_D);
        preferences.putFloat("P_DST_LIM", currentSettings.PID_DIST_LIM);
        preferences.putFloat("P_DST_ILM", currentSettings.PID_DIST_ILIM);
        preferences.putFloat("P_DST_DED", currentSettings.PID_DIST_DEAD);

        preferences.putFloat("P_OBS_P", currentSettings.PID_OBSTACLE_P);
        preferences.putFloat("P_OBS_I", currentSettings.PID_OBSTACLE_I);
        preferences.putFloat("P_OBS_D", currentSettings.PID_OBSTACLE_D);
        preferences.putFloat("P_OBS_LIM", currentSettings.PID_OBSTACLE_LIM);
        preferences.putFloat("P_OBS_ILM", currentSettings.PID_OBSTACLE_ILIM);
        preferences.putFloat("P_OBS_DED", currentSettings.PID_OBSTACLE_DEAD);

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

        else if (varName == "SERIAL_DEBUG_MASTER") { currentSettings.SERIAL_DEBUG_MASTER = FactoryDefaults::SERIAL_DEBUG_MASTER; }*/


        if (varName == "CRUISING_SPEED") { currentSettings.CRUISING_SPEED = FactoryDefaults::CRUISING_SPEED; }
        else if (varName == "OBSTACLE_TRIGGER_CM") { currentSettings.OBSTACLE_TRIGGER_CM = FactoryDefaults::OBSTACLE_TRIGGER_CM; }
        else if (varName == "MAINTAIN_DISTANCE_CM") { currentSettings.MAINTAIN_DISTANCE_CM = FactoryDefaults::MAINTAIN_DISTANCE_CM; }
        else if (varName == "MOTOR_MIN_PWM") { currentSettings.MOTOR_MIN_PWM = FactoryDefaults::MOTOR_MIN_PWM; }

        else if (varName == "OBS_SWEEP_ANGLE") { currentSettings.OBS_SWEEP_ANGLE = FactoryDefaults::OBS_SWEEP_ANGLE; }
        else if (varName == "OBS_SWEEP_SPEED") { currentSettings.OBS_SWEEP_SPEED = FactoryDefaults::OBS_SWEEP_SPEED; }
        else if (varName == "OBS_SWEEP_PAUSE") { currentSettings.OBS_SWEEP_PAUSE = FactoryDefaults::OBS_SWEEP_PAUSE; }
        else if (varName == "OBS_CLEAR_THRESH") { currentSettings.OBS_CLEAR_THRESH = FactoryDefaults::OBS_CLEAR_THRESH; }
        else if (varName == "OBS_HYSTERESIS") { currentSettings.OBS_HYSTERESIS = FactoryDefaults::OBS_HYSTERESIS; }

        else if (varName == "DIZZY_SPIN_PWM") { currentSettings.DIZZY_SPIN_PWM = FactoryDefaults::DIZZY_SPIN_PWM; }
        else if (varName == "DIZZY_SPIN_TIME") { currentSettings.DIZZY_SPIN_TIME = FactoryDefaults::DIZZY_SPIN_TIME; }
        else if (varName == "DIZZY_COOLDOWN") { currentSettings.DIZZY_COOLDOWN = FactoryDefaults::DIZZY_COOLDOWN; }
        else if (varName == "SLEEP_TIMEOUT_MS") { currentSettings.SLEEP_TIMEOUT_MS = FactoryDefaults::SLEEP_TIMEOUT_MS; }
        else if (varName == "SLEEP_WAKE_G") { currentSettings.SLEEP_WAKE_G = FactoryDefaults::SLEEP_WAKE_G; }

        else if (varName == "IMU_MADGWICK_BETA") { currentSettings.IMU_MADGWICK_BETA = FactoryDefaults::IMU_MADGWICK_BETA; }
        else if (varName == "IMU_GYRO_DEADBAND") { currentSettings.IMU_GYRO_DEADBAND = FactoryDefaults::IMU_GYRO_DEADBAND; }
        else if (varName == "SONAR_MAX_DIST") { currentSettings.SONAR_MAX_DIST = FactoryDefaults::SONAR_MAX_DIST; }

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

        else if (varName == "PID_DIST_P") { currentSettings.PID_DIST_P = FactoryDefaults::PID_DIST_P; }
        else if (varName == "PID_DIST_I") { currentSettings.PID_DIST_I = FactoryDefaults::PID_DIST_I; }
        else if (varName == "PID_DIST_D") { currentSettings.PID_DIST_D = FactoryDefaults::PID_DIST_D; }
        else if (varName == "PID_DIST_LIM") { currentSettings.PID_DIST_LIM = FactoryDefaults::PID_DIST_LIM; }
        else if (varName == "PID_DIST_ILIM") { currentSettings.PID_DIST_ILIM = FactoryDefaults::PID_DIST_ILIM; }
        else if (varName == "PID_DIST_DEAD") { currentSettings.PID_DIST_DEAD = FactoryDefaults::PID_DIST_DEAD; }

        else if (varName == "PID_OBSTACLE_P") { currentSettings.PID_OBSTACLE_P = FactoryDefaults::PID_OBSTACLE_P; }
        else if (varName == "PID_OBSTACLE_I") { currentSettings.PID_OBSTACLE_I = FactoryDefaults::PID_OBSTACLE_I; }
        else if (varName == "PID_OBSTACLE_D") { currentSettings.PID_OBSTACLE_D = FactoryDefaults::PID_OBSTACLE_D; }
        else if (varName == "PID_OBSTACLE_LIM") { currentSettings.PID_OBSTACLE_LIM = FactoryDefaults::PID_OBSTACLE_LIM; }
        else if (varName == "PID_OBSTACLE_ILIM") { currentSettings.PID_OBSTACLE_ILIM = FactoryDefaults::PID_OBSTACLE_ILIM; }
        else if (varName == "PID_OBSTACLE_DEAD") { currentSettings.PID_OBSTACLE_DEAD = FactoryDefaults::PID_OBSTACLE_DEAD; }

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