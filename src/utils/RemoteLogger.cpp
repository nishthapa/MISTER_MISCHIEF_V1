#include "utils/RemoteLogger.h"
#include "config/SystemConfig.h"
#include "core/GlobalDataBus.h"
#include "config/ConfigurationManager.h"

RemoteLogger::RemoteLogger() : currentMode(0) {}

void RemoteLogger::beginSerial() {
    currentMode = SysConfig.ACTIVE_DEBUG_MODE;
    if (currentMode & SysConfig.DEBUG_USB) {
        Serial.begin(SysConfig.SERIAL_BAUD_RATE);
        delay(3000);
    }
}

void RemoteLogger::bindRadios() {
    if (currentMode & SysConfig.DEBUG_USB) {
        Serial.println("=== TELEMETRY ROUTER ONLINE ===");
        Serial.println("[USB] ONLINE");
        Serial.print("[WIFI] ");
        if (currentMode & SysConfig.DEBUG_WIFI & SysConfig.WIFI_ACTIVE) Serial.println("ROUTED VIA TELEMETRY STREAMER");
        else Serial.println("OFF / UNAVAILABLE");
        Serial.println("===============================\n");
    }
}

void RemoteLogger::print(const char* message) {
    if ((currentMode & SysConfig.DEBUG_USB) && Serial) Serial.print(message);
    if (currentMode & SysConfig.DEBUG_WIFI) {
        portENTER_CRITICAL(&globalDataBusLock);
        strncpy(CurrentRobotData.systemLog.text, message, 127);
        CurrentRobotData.systemLog.text[127] = '\0';
        CurrentRobotData.hasNewLog = true;
        portEXIT_CRITICAL(&globalDataBusLock);
    }
}

void RemoteLogger::println(const char* message) {
    if ((currentMode & SysConfig.DEBUG_USB) && Serial) Serial.println(message);
    if (currentMode & SysConfig.DEBUG_WIFI) {
        portENTER_CRITICAL(&globalDataBusLock);
        strncpy(CurrentRobotData.systemLog.text, message, 126);
        CurrentRobotData.systemLog.text[126] = '\0'; 
        strcat(CurrentRobotData.systemLog.text, "\n");
        CurrentRobotData.hasNewLog = true;
        portEXIT_CRITICAL(&globalDataBusLock);
    }
}

void RemoteLogger::printf(const char* format, ...) {
    if (currentMode == 0) return;
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    print(buffer);
}

void RemoteLogger::print(const String& message) { print(message.c_str()); }
void RemoteLogger::println(const String& message) { println(message.c_str()); }