#include "tasks/Task_Network.h"
#include "core/GlobalDataBus.h"
#include "config/SystemConfig.h"
#include "utils/RemoteLogger.h"

extern RemoteLogger logger;

// ==========================================
// TASK 3: NETWORK & TELEMETRY PUBLISHER
// (CORE 1 - APP CPU)
// ==========================================

void NetworkTask(void *pvParameters) {
    NetworkContext* ctx = static_cast<NetworkContext*>(pvParameters);
    unsigned long lastTelemetryTime = 0;

    for (;;) {
        // 1. Read CLI input securely
        while (Serial.available()) {
            if (ctx->cli) ctx->cli->processChar(Serial.read());
        }

        // 2. Handle incoming WebSocket handshakes 
        logger.handleClient();

        // 3. BROADCAST BINARY TELEMETRY
        unsigned long currentTime = millis();
        
        if (currentTime - lastTelemetryTime >= SystemConfig::TELEMETRY_PING_DELAY_MS) {
            lastTelemetryTime = currentTime;

            // --- THE SNAPSHOT SPINLOCK ---
            GlobalDataBank snapshot; 
            
            portENTER_CRITICAL(&globalDataBusLock);
            snapshot = CurrentRobotData; 
            portEXIT_CRITICAL(&globalDataBusLock);
            // -----------------------------

            if (ctx->router) {
                // Blast Attitude Data (Pitch, Roll, Yaw, Motors)
                ctx->router->broadcast(Comms::MsgId::ATTITUDE, snapshot.physics);

                // Blast Left & Right Motor PWMs
                ctx->router->broadcast(Comms::MsgId::ACTUATORS, snapshot.physics);

                // Blast System Health (Loop time, Heap, Hardware Bitmask, RSSI)
                ctx->router->broadcast(Comms::MsgId::SYSTEM_STATUS, snapshot.health);

                // Blast Cognitive State (Mode & Mood)
                ctx->router->broadcast(Comms::MsgId::ROBOT_STATE, snapshot.cognition);

                // Blast Sensors (Sonar distance)
                ctx->router->broadcast(Comms::MsgId::DISTANCE_SONAR, snapshot.sensors);

                // Blast Semantic Events (latches & Continuous Metrics
                ctx->router->broadcast(Comms::MsgId::EVENT_STATE, snapshot.events);

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
        
        // Give the network stack breathing room
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}