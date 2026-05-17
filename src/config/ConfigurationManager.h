#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "config/FactoryDefaults.h" // <-- Include our new master defaults

struct MasterSettings {
    // Initialize using the Master Defaults
    float CRUISING_SPEED = FactoryDefaults::CRUISING_SPEED;
    float OBSTACLE_TRIGGER_CM = FactoryDefaults::OBSTACLE_TRIGGER_CM;
    float MAINTAIN_DIST_CM = FactoryDefaults::MAINTAIN_DIST_CM;
    
    bool BRAIN_ACTIVE = FactoryDefaults::BRAIN_ACTIVE;

    // --- Network & Comms ---
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
        currentSettings.MAINTAIN_DIST_CM = preferences.getFloat("DIST_CM", FactoryDefaults::MAINTAIN_DIST_CM);
        
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
        preferences.putFloat("DIST_CM", currentSettings.MAINTAIN_DIST_CM);
        
        preferences.putBool("BRAIN_ACT", currentSettings.BRAIN_ACTIVE);

        preferences.putString("WIFI_SSID", currentSettings.WIFI_SSID);
        preferences.putString("WIFI_PASS", currentSettings.WIFI_PASSWORD);
        preferences.putBool("WIFI_ACT", currentSettings.WIFI_ACTIVE);
        
        preferences.putString("BT_NAME", currentSettings.BT_NAME);
        preferences.putBool("BT_ACT", currentSettings.BT_ACTIVE);

        preferences.putBool("DBG_MASTER", currentSettings.SERIAL_DEBUG_MASTER);
        preferences.putBool("DBG_IMU", currentSettings.SERIAL_DEBUG_IMU);
        preferences.putBool("DBG_SONAR", currentSettings.SERIAL_DEBUG_SONAR);
        preferences.putBool("DBG_MOTOR", currentSettings.SERIAL_DEBUG_MOTOR_DRIVER);
    }

    // --- NEW: Granular Reset ---
    bool resetVariable(String varName) {
        varName.toUpperCase();
        if (varName == "CRUISING_SPEED") { currentSettings.CRUISING_SPEED = FactoryDefaults::CRUISING_SPEED; }
        else if (varName == "OBSTACLE_TRIGGER_CM") { currentSettings.OBSTACLE_TRIGGER_CM = FactoryDefaults::OBSTACLE_TRIGGER_CM; }
        else if (varName == "MAINTAIN_DIST_CM") { currentSettings.MAINTAIN_DIST_CM = FactoryDefaults::MAINTAIN_DIST_CM; }

        else if (varName == "BRAIN_ACTIVE") { currentSettings.BRAIN_ACTIVE = FactoryDefaults::BRAIN_ACTIVE; }

        else if (varName == "WIFI_SSID") { currentSettings.WIFI_SSID = FactoryDefaults::WIFI_SSID; }
        else if (varName == "WIFI_PASSWORD") { currentSettings.WIFI_PASSWORD = FactoryDefaults::WIFI_PASSWORD; }
        else if (varName == "WIFI_ACTIVE") { currentSettings.WIFI_ACTIVE = FactoryDefaults::WIFI_ACTIVE; }
        
        else if (varName == "BT_NAME") { currentSettings.BT_NAME = FactoryDefaults::BT_NAME; }
        else if (varName == "BT_ACTIVE") { currentSettings.BT_ACTIVE = FactoryDefaults::BT_ACTIVE; }

        else if (varName == "SERIAL_DEBUG_MASTER") { currentSettings.SERIAL_DEBUG_MASTER = FactoryDefaults::SERIAL_DEBUG_MASTER; }
        else { return false; } // Variable not found
        save();
        return true;
    }

    void factoryReset() {
        preferences.clear(); 
        currentSettings = MasterSettings(); // Struct re-initializes to defaults
        save(); 
    }
};

#define Config ConfigurationManager::getInstance().get()
#define ConfigSys ConfigurationManager::getInstance()