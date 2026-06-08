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
        uint8_t packetBuffer[64]; 

    public:
        TelemetryStreamer() {}

        // Registers a medium (e.g., WiFi, BLE, USB) to the router
        void registerSink(ITelemetrySink* sink) {
            if (sinkCount < MAX_SINKS) {
                activeSinks[sinkCount] = sink;
                sinkCount++;
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
    };

} // namespace Comms