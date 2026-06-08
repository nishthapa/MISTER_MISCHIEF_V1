#pragma once

#include <Arduino.h>
#include "config/PacketIDRegistry.h"

namespace Comms {

    class BinaryPacketBuilder {
    public:
        // Template function: Accepts any struct, calculates its size automatically, 
        // and builds the exact MultiWii-style binary packet.
        template <typename T>
        static size_t buildPacket(MsgId id, const T& payloadStruct, uint8_t* outBuffer) {
            uint8_t payloadSize = sizeof(T);
            
            // 1. The Header ($ M >)
            outBuffer[0] = '$';
            outBuffer[1] = 'M';
            outBuffer[2] = '>'; // Direction: Robot to App
            
            // 2. Size and ID
            outBuffer[3] = payloadSize;
            outBuffer[4] = static_cast<uint8_t>(id);
            
            // 3. The Payload (Instantly copies the raw memory of the struct into the buffer)
            memcpy(&outBuffer[5], &payloadStruct, payloadSize);
            
            // 4. The XOR Checksum calculation
            // Checksum = Size XOR ID XOR [Every Payload Byte]
            uint8_t checksum = outBuffer[3] ^ outBuffer[4];
            for (uint8_t i = 0; i < payloadSize; i++) {
                checksum ^= outBuffer[5 + i];
            }
            
            // Append checksum to the very end
            outBuffer[5 + payloadSize] = checksum;
            
            // Return total packet length (Header + Size + ID + Payload + Checksum)
            return 6 + payloadSize; 
        }
    };

} // namespace Comms