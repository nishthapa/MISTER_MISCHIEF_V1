#include "utils/OTAManager.h"
#include "utils/RemoteLogger.h"
#include "core/GlobalDataBus.h"
#include <ArduinoOTA.h>

extern RemoteLogger logger;

void OTAManager::begin() {
    ArduinoOTA.setHostname("MisterMischief");

    ArduinoOTA.onStart([]() {
        String type = (ArduinoOTA.getCommand() == U_FLASH) ? "firmware" : "filesystem";
        logger.print("\n[OTA] Update Started: ");
        logger.println(type.c_str());
        logger.println("[OTA] Alerting RTOS to gracefully halt physics...");

        // 🚨 THROW THE GLOBAL FLAG! 
        // This tells the other cores to shut off the motors and go to sleep.
        portENTER_CRITICAL(&globalDataBusLock);
        CurrentRobotData.otaUpdateStarted = true;
        portEXIT_CRITICAL(&globalDataBusLock);
    });

    ArduinoOTA.onEnd([]() {
        logger.println("\n[OTA] Update Success! Rebooting...");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        // Print strictly to USB to avoid flooding the WebSocket during flashing
        Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) logger.println("[OTA] Auth Failed");
        else if (error == OTA_BEGIN_ERROR) logger.println("[OTA] Begin Failed");
        else if (error == OTA_CONNECT_ERROR) logger.println("[OTA] Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) logger.println("[OTA] Receive Failed");
        else if (error == OTA_END_ERROR) logger.println("[OTA] End Failed");
    });

    ArduinoOTA.begin();
}

void OTAManager::handle() {
    ArduinoOTA.handle();
}