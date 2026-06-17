#pragma once
#include "ITelemetrySink.h"
#include "BinaryPacketBuilder.h"
#include "core/GlobalDataBus.h" // For access to CurrentRobotData

namespace Comms {

    class TelemetryStreamer {
    private:
        static constexpr int MAX_SINKS = 4;
        ITelemetrySink* activeSinks[MAX_SINKS];
        uint8_t sinkCount = 0;

        // A reusable buffer (64 bytes is plenty for our structs)
        // Prevents any dynamic memory allocation (no heap fragmentation!)
        // CRITICAL FIX: Increased from 64 to 150 to safely hold 128-byte text logs + packet headers!
        uint8_t packetBuffer[150]; // Increased for text logs!

    public:
        TelemetryStreamer() {}

        // Registers a medium (e.g., WiFi, BLE, USB) to the router
        void registerSink(ITelemetrySink* sink) {
            if (sinkCount < MAX_SINKS) {
                activeSinks[sinkCount] = sink;
                sinkCount++;
            }
        }

        // NEW: Let the Streamer run the background loops for all sinks
        void pollSinks() {
            for (uint8_t i = 0; i < sinkCount; i++) {
                activeSinks[i]->update();
            }
        }

        // The Smart Router: Builds the packet ONCE, blasts it to everyone active
        template <typename T>
        void broadcast(MsgId id, const T& dataStruct) {
            // 1. Build the binary envelope instantly
            size_t packetSize = BinaryPacketBuilder::buildPacket(id, dataStruct, packetBuffer);

            // 2. Route it to all connected hardware
            for (uint8_t i = 0; i < sinkCount; i++) {
                // isReady() checks natively if the hardware is connected (e.g. BLE has a client)
                if (activeSinks[i]->isReady()) { 
                    activeSinks[i]->sendBinary(packetBuffer, packetSize);
                }
            }
        }

        void broadcastString(MsgId id, const char* text) {
            uint8_t len = strlen(text);
            if (len == 0) return;
            
            // Hard cap the length just in case, preventing buffer overflows!
            if (len > 128) len = 128;

            // Build the packet instantly
            size_t packetSize = BinaryPacketBuilder::buildStringPacket(id, text, len, packetBuffer);

            // Route it to all connected hardware
            for (uint8_t i = 0; i < sinkCount; i++) {
                if (activeSinks[i]->isReady()) {
                    activeSinks[i]->sendBinary(packetBuffer, packetSize);
                }
            }
        }
    };

} // namespace Comms