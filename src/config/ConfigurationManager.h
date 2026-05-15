#pragma once
#include <Arduino.h>
#include <Preferences.h>

// 1. THE RAM STRUCT: This holds all the live variables the robot uses to think. [cite: 1]
// (We will add the PID variables and Motor variables to this struct later!) [cite: 2]
struct MasterSettings {
    // --- Personality Settings --- [cite: 2]
    float CRUISING_SPEED = 30.0f;
    float OBSTACLE_TRIGGER_CM = 30.0f;
    float MAINTAIN_DIST_CM = 15.0f;
    
    // --- System States --- [cite: 3]
    bool BRAIN_ACTIVE = true; // Lets you freeze the robot's physical movements [cite: 4]
    
    // --- Debug & Telemetry --- [cite: 4]
    bool SERIAL_DEBUG_MASTER = true; // The Master Killswitch [cite: 5]
    bool SERIAL_DEBUG_IMU = false;   // Granular IMU debug (for later) [cite: 6]
    bool SERIAL_DEBUG_SONAR = false; // Granular Sonar debug (for later) 
    bool SERIAL_DEBUG_MOTOR_DRIVER = false; // Granular Motor Driver debug (for later) 
};

// 2. THE HARD DRIVE MANAGER [cite: 8]
class ConfigurationManager {
private:
    Preferences preferences;
    MasterSettings currentSettings;

public:
    // Standard Singleton Pattern (so all files can talk to the exact same hard drive) [cite: 9]
    static ConfigurationManager& getInstance() {
        static ConfigurationManager instance;
        return instance;
    }

    void init() {
        // Open the "mischief" partition in read/write mode [cite: 10]
        preferences.begin("mischief", false);

        // Load explicitly by Key. If the key doesn't exist, it falls back to the default!
        currentSettings.CRUISING_SPEED = preferences.getFloat("CRUISE_SPD", 30.0f);
        currentSettings.OBSTACLE_TRIGGER_CM = preferences.getFloat("OBS_TRIG_CM", 30.0f);
        currentSettings.MAINTAIN_DIST_CM = preferences.getFloat("DIST_CM", 15.0f);
        
        currentSettings.BRAIN_ACTIVE = preferences.getBool("BRAIN_ACT", true);
        currentSettings.SERIAL_DEBUG_MASTER = preferences.getBool("DBG_MASTER", true);
        currentSettings.SERIAL_DEBUG_IMU = preferences.getBool("DBG_IMU", false);
        currentSettings.SERIAL_DEBUG_SONAR = preferences.getBool("DBG_SONAR", false);
        currentSettings.SERIAL_DEBUG_MOTOR_DRIVER = preferences.getBool("DBG_MOTOR", false);
    }

    // Get the live variables [cite: 13]
    MasterSettings& get() { return currentSettings; }

    // Save the live variables to the hard drive [cite: 14]
    void save() {
        // Save explicitly by Key. Bulletproof across firmware updates!
        preferences.putFloat("CRUISE_SPD", currentSettings.CRUISING_SPEED);
        preferences.putFloat("OBS_TRIG_CM", currentSettings.OBSTACLE_TRIGGER_CM);
        preferences.putFloat("DIST_CM", currentSettings.MAINTAIN_DIST_CM);
        
        preferences.putBool("BRAIN_ACT", currentSettings.BRAIN_ACTIVE);
        preferences.putBool("DBG_MASTER", currentSettings.SERIAL_DEBUG_MASTER);
        preferences.putBool("DBG_IMU", currentSettings.SERIAL_DEBUG_IMU);
        preferences.putBool("DBG_SONAR", currentSettings.SERIAL_DEBUG_SONAR);
        preferences.putBool("DBG_MOTOR", currentSettings.SERIAL_DEBUG_MOTOR_DRIVER);
    }

    // The Nuclear Option [cite: 15]
    void factoryReset() {
        preferences.clear(); // Wipe the partition [cite: 16]
        currentSettings = MasterSettings(); // Reset RAM to defaults [cite: 17]
        save(); // Save defaults [cite: 17]
    }
};

// 3. THE GLOBAL SHORTCUTS [cite: 18]
// "Config" gives you the variables: Config.CRUISING_SPEED [cite: 18]
// "ConfigSys" gives you the system: ConfigSys.save() [cite: 18]
#define Config ConfigurationManager::getInstance().get()
#define ConfigSys ConfigurationManager::getInstance()