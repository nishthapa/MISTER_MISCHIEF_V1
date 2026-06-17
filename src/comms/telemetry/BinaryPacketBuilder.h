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
            // memcpy(&outBuffer[5], &payloadStruct, payloadSize);

            // For now, we cast to (const void*) to strip the FreeRTOS 'volatile' tag for the microsecond it takes to copy

            // TO-DO: we must completely delete the volatile keyword from the entire project and
            // replace it with a FreeRTOS Hardware Spinlock (portMUX_TYPE).
            memcpy(&outBuffer[5], (const void*)&payloadStruct, payloadSize);  
            
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

        // NEW: Highly efficient variable-length string packet builder
        static size_t buildStringPacket(MsgId id, const char* text, uint8_t len, uint8_t* outBuffer) {
            // 1. The Header ($ M >)
            outBuffer[0] = '$';
            outBuffer[1] = 'M';
            outBuffer[2] = '>'; 
            
            // 2. Size and ID
            outBuffer[3] = len;
            outBuffer[4] = static_cast<uint8_t>(id);
            
            // 3. The Payload (Copy just the active string bytes)
            memcpy(&outBuffer[5], text, len);  
            
            // 4. The Checksum calculation
            uint8_t checksum = outBuffer[3] ^ outBuffer[4];
            for (uint8_t i = 0; i < len; i++) {
                checksum ^= outBuffer[5 + i];
            }
            
            // Append checksum
            outBuffer[5 + len] = checksum;
            
            // Return total packet length (Header[3] + Size[1] + ID[1] + Payload + Checksum[1])
            return 5 + len + 1;
        }
    };

} // namespace Comms