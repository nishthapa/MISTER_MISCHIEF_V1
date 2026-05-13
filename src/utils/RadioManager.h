#pragma once
#include <WiFi.h>

class RadioManager {
public:
    // Boots whatever radios are requested in the Config files
    static void initRadios();
    
    // Allows any service (Logger, Web Server, App) to check if the pipes are open
    static bool isWiFiReady();
    static bool isBluetoothReady();
};