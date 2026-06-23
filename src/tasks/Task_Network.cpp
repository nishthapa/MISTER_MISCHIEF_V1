#include "tasks/Task_Network.h"
#include "core/GlobalDataBus.h"
#include "config/SystemConfig.h"
#include "utils/RemoteLogger.h"
#include "utils/OTAManager.h" // for OTA flashing
#include <Wifi.h>

extern RemoteLogger logger;

// ==========================================
// TASK 3: NETWORK & TELEMETRY PUBLISHER
// (CORE 1 - APP CPU)
// ==========================================

void NetworkTask(void *pvParameters) {
    NetworkContext* ctx = static_cast<NetworkContext*>(pvParameters);
    unsigned long lastTelemetryTime = 0;

    // STATE VARIABLES FOR WIFI RSSI
    unsigned long lastRSSITime = 0;      
    int8_t currentWifiRSSI = -127;

    for (;;) {
        if (Serial) {
            portENTER_CRITICAL(&globalDataBusLock);
            CurrentRobotData.health.hardwareBitmask |= Comms::HealthBit::USB_CONNECTED;
            portEXIT_CRITICAL(&globalDataBusLock);
        } else {
            portENTER_CRITICAL(&globalDataBusLock);
            CurrentRobotData.health.hardwareBitmask &= ~Comms::HealthBit::USB_CONNECTED;
            portEXIT_CRITICAL(&globalDataBusLock);
        }
        
        // 1. Read CLI input securely
        while (Serial.available()) {
            if (ctx->cli) ctx->cli->processChar(Serial.read());
        }

        // Poll the hardware radio for live signal strength
        //portENTER_CRITICAL(&globalDataBusLock);
        if (WiFi.status() == WL_CONNECTED) {
            portENTER_CRITICAL(&globalDataBusLock);
            CurrentRobotData.health.hardwareBitmask |= Comms::HealthBit::WIFI_CONNECTED;
            //CurrentRobotData.networkLink.wifiRSSI = WiFi.RSSI();
            portEXIT_CRITICAL(&globalDataBusLock);
        } else {
            portENTER_CRITICAL(&globalDataBusLock);
            CurrentRobotData.health.hardwareBitmask &= ~Comms::HealthBit::WIFI_CONNECTED;
            CurrentRobotData.networkLink.wifiRSSI = -127;
            portEXIT_CRITICAL(&globalDataBusLock);
        }
        
        // Note: NimBLE acts as a Server. Live BLE RSSI requires complex client-side callbacks,
        // so we just set it to 0 (Perfect) if connected, and -127 (Dead) if not.
        if (CurrentRobotData.health.hardwareBitmask & Comms::HealthBit::BLE_CONNECTED) {
            portENTER_CRITICAL(&globalDataBusLock);
            CurrentRobotData.networkLink.bleRSSI = 0; 
            portEXIT_CRITICAL(&globalDataBusLock);
        } else {
            portENTER_CRITICAL(&globalDataBusLock);
            CurrentRobotData.networkLink.bleRSSI = -127;
            portEXIT_CRITICAL(&globalDataBusLock);
        }
        //portEXIT_CRITICAL(&globalDataBusLock);

        // 2. RUN THE NETWORK HEARTBEATS (This runs ws.loop() safely!)
        if (ctx->router) {
            ctx->router->pollSinks();
        }

        // 3. BROADCAST CONTINUOUS BINARY TELEMETRY
        unsigned long currentTime = millis();

        // ==========================================
        // 🚨 MEASURE LIVE SIGNAL STRENGTH (RSSI)
        // EVERY 2 SECONDS
        // ==========================================
        
        
        if (currentTime - lastTelemetryTime >= SystemConfig::TELEMETRY_PING_DELAY_MS) {
            lastTelemetryTime = currentTime;
            
            // --- THE SNAPSHOT SPINLOCK ---
            GlobalDataBank snapshot; 
            portENTER_CRITICAL(&globalDataBusLock);
            snapshot = CurrentRobotData; 
            portEXIT_CRITICAL(&globalDataBusLock);
            // -----------------------------

            if (ctx->router) {
                // Blast Cognitive State (Mode & Mood)
                ctx->router->broadcast(Comms::MsgId::ROBOT_STATE, snapshot.cognition);

                // Blast Attitude Data (Pitch, Roll, Yaw, Motors)
                ctx->router->broadcast(Comms::MsgId::ATTITUDE, snapshot.physics);

                // Blast Sensors (Sonar distance)
                ctx->router->broadcast(Comms::MsgId::DISTANCE_SONAR, snapshot.sensors);

                // Blast System Health (Loop time, Heap, Hardware Bitmask, RSSI)
                ctx->router->broadcast(Comms::MsgId::SYSTEM_STATUS, snapshot.health);

                // Blast Serial.println() messages handled by RemoteLogger
                //ctx->router->broadcast(Comms::MsgId::TRANSIENT_ALERTS, snapshot.systemLog); // shouldnt be blasted continuously!

                // Blast Remote Joystick & PID enabled/disabled flag
                ctx->router->broadcast(Comms::MsgId::CONTROL_DEBUG, snapshot.controlDebug);

                // Blast Left & Right Motor PWMs
                ctx->router->broadcast(Comms::MsgId::ACTUATORS, snapshot.actuators);
                
                // Blast Semantic Events (latches & Continuous Metrics
                ctx->router->broadcast(Comms::MsgId::EVENT_STATE, snapshot.events);

                // Blast Network Link (Signal strength)
                ctx->router->broadcast(Comms::MsgId::NETWORK_LINK, snapshot.networkLink);

                // Blast Perception data (raw energies)
                ctx->router->broadcast(Comms::MsgId::PERCEPTION_METRICS, snapshot.perception);
            }

            //experimental: just to view in serial monitor for now
            // char debugMsg[128];
            // snprintf(debugMsg, sizeof(debugMsg), "Sonar: %.1f cm | Yaw: %.1f° | Pitch: %.1f° | Roll: %.1f° | L_MOTOR_PWM: %d | R_MOTOR_PWM: %d | MODE: %s", 
            //          snapshot.sensors.distanceCM, 
            //          snapshot.physics.imuAngles.yaw, 
            //          snapshot.physics.imuAngles.pitch, 
            //          snapshot.physics.imuAngles.roll,
            //          snapshot.physics.leftMotorPWM,
            //          snapshot.physics.rightMotorPWM,
            //          ctx->brain ? ctx->brain->getActiveModeName() : "UNKNOWN");
                     
            // logger.println(debugMsg);
        }

        // ==========================================
        // 4. CHECK FOR NEW TEXT LOGS (TRANSIENT ALERTS)
        // ==========================================
        bool shouldBroadcastLog = false;
        char localLogBuffer[128] = {0};
        
        portENTER_CRITICAL(&globalDataBusLock);
        // If RemoteLogger dropped a message onto the bus...
        if (CurrentRobotData.hasNewLog) {
            shouldBroadcastLog = true;
            // Safely copy it out
            strncpy(localLogBuffer, CurrentRobotData.systemLog.text, 127);
            localLogBuffer[127] = '\0'; // Guarantee termination 
            CurrentRobotData.hasNewLog = false; // Lower the flag
        }
        portEXIT_CRITICAL(&globalDataBusLock);

        if (shouldBroadcastLog && ctx->router) {
            // Uses your custom ID 140!
            // This safely calculates the string length and ONLY sends the required bytes!
            ctx->router->broadcastString(Comms::MsgId::TRANSIENT_ALERTS, localLogBuffer);
        }

        // ==========================================
        // 5. HANDLE OTA UPDATES
        // ==========================================
        OTAManager::handle();
        
        // Give the network stack breathing room
        vTaskDelay(pdMS_TO_TICKS(SystemConfig::MAIN_LOOP_TICK_RATE_MS));
    }
}