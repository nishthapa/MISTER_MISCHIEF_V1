import asyncio
import json
import time
import struct
from bleak import BleakClient # Removed BleakScanner
from foxglove_websocket.server import FoxgloveServer

# 1. Swap the Name for the exact Hardware MAC
ROBOT_MAC_ADDRESS = "44:1B:F6:FF:94:19" 
CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

# =========================================================================
# C++ STRUCT BYTE MAPPINGS (Little Endian format)
# =========================================================================
FMT_SYSTEM_HEALTH = "<IIHbb" 
FMT_NETWORK_LINK = "<bb"
FMT_SENSOR_STATE = "<fHh"
FMT_EVENT_STATE = "<8?5f7?"
FMT_COGNITIVE = "<BB"
FMT_CONTROL_DEBUG = "<ff"
FMT_PERCEPTION = "<6f"
FMT_PHYSICS = "<ffff?fhh"

class BLEBridge:
    def __init__(self):
        self.server = None
        self.channels = {}
        self.byte_buffer = bytearray()

    async def start(self):
        async with FoxgloveServer("0.0.0.0", 8765, "Mister Mischief BLE Bridge", supported_encodings=["json"]) as server:
            self.server = server
            
            # --- REGISTER FOXGLOVE CHANNELS ---
            schemas = {
                "logs": "Logs", "system_health": "SystemHealth", "network_link": "NetworkLink",
                "sensors": "Sensors", "events": "EventState", "cognition": "CognitiveState",
                "control_debug": "ControlDebug", "perception": "PerceptionMetrics", "physics": "PhysicsState"
            }
            for topic, schema_name in schemas.items():
                self.channels[topic] = await server.add_channel({
                    "topic": f"robot/{topic}", "encoding": "json", 
                    "schemaName": schema_name, "schema": json.dumps({"type": "object"})
                })

            while True:
                print(f"\nAttempting direct connection to {ROBOT_MAC_ADDRESS}...")
                
                try:
                    # 2. Directly connect using the MAC address
                    async with BleakClient(ROBOT_MAC_ADDRESS, timeout=10.0) as client:
                        print("Connected! Negotiating MTU and subscribing to Telemetry Stream...")
                        
                        # Subscribe to the C++ NOTIFY characteristic
                        await client.start_notify(CHAR_UUID, self.ble_notification_handler)
                        print("Telemetry Flowing -> Relaying to Foxglove Studio...")
                        
                        # Keep the connection alive until the robot disconnects
                        while client.is_connected:
                            await asyncio.sleep(1)
                            
                except Exception as e:
                    # 3. Graceful fallback if the robot is off or out of range
                    print(f"BLE Link Dropped/Failed: {e}")
                    self.byte_buffer.clear() # Flush corrupt data on disconnect
                    print("Retrying in 2 seconds...")
                    await asyncio.sleep(2)

    def ble_notification_handler(self, sender, data):
        # 1. Append raw radio bytes to the sliding window buffer
        self.byte_buffer.extend(data)
        
        # 2. Extract and reassemble MultiWii Packets
        while len(self.byte_buffer) >= 6:
            # Check Header: '$' (36), 'M' (77), '>' (62)
            if self.byte_buffer[0] != 36 or self.byte_buffer[1] != 77 or self.byte_buffer[2] != 62:
                self.byte_buffer.pop(0) # Slide window forward 1 byte
                continue
                
            size = self.byte_buffer[3]
            packet_len = 6 + size
            
            if len(self.byte_buffer) < packet_len:
                break # Wait for the rest of the packet to arrive over radio
                
            msg_id = self.byte_buffer[4]
            payload = self.byte_buffer[5 : 5 + size]
            checksum = self.byte_buffer[5 + size]
            
            # Verify XOR Checksum
            calc_checksum = size ^ msg_id
            for b in payload:
                calc_checksum ^= b
                
            if calc_checksum == checksum:
                # Fire an async task to route to Foxglove without blocking the radio listener
                asyncio.create_task(self.route_to_foxglove(msg_id, payload))
            else:
                print(f"BLE Radio Interference: Checksum mismatch on ID {msg_id}! Dropped.")
                
            # Consume the processed packet from the buffer
            self.byte_buffer = self.byte_buffer[packet_len:]

    async def route_to_foxglove(self, msg_id, payload):
        timestamp_nanos = time.time_ns()
        
        try:
            if msg_id == 140:
                text = payload.decode('utf-8', errors='ignore')
                await self.server.send_message(self.channels["logs"], timestamp_nanos, json.dumps({"log": text}).encode("utf8"))
                print(f"[ROBOT] {text}")
            
            elif msg_id == 130 and len(payload) == struct.calcsize(FMT_SYSTEM_HEALTH):
                data = struct.unpack(FMT_SYSTEM_HEALTH, payload)
                doc = {"loopTimeUs": data[0], "freeHeap": data[1], "hardwareBitmask": data[2], "wifiRSSI": data[3], "bleRSSI": data[4]}
                await self.server.send_message(self.channels["system_health"], timestamp_nanos, json.dumps(doc).encode("utf8"))

            elif msg_id == 132 and len(payload) == struct.calcsize(FMT_NETWORK_LINK):
                data = struct.unpack(FMT_NETWORK_LINK, payload)
                await self.server.send_message(self.channels["network_link"], timestamp_nanos, json.dumps({"wifiRSSI": data[0], "bleRSSI": data[1]}).encode("utf8"))

            elif msg_id == 110 and len(payload) == struct.calcsize(FMT_SENSOR_STATE):
                data = struct.unpack(FMT_SENSOR_STATE, payload)
                doc = {"distanceCM": round(data[0], 2), "batteryVoltageMV": data[1], "currentDrawMA": data[2]}
                await self.server.send_message(self.channels["sensors"], timestamp_nanos, json.dumps(doc).encode("utf8"))

            elif msg_id == 135 and len(payload) == struct.calcsize(FMT_EVENT_STATE):
                data = struct.unpack(FMT_EVENT_STATE, payload)
                doc = {
                    "hazardDetected": data[0], "teaseConfirmed": data[1], "targetVanished": data[2], 
                    "dizzyTriggered": data[3], "dizzyFinished": data[4], "readyForCompassLock": data[5], 
                    "safelyLanded": data[6], "frustrationPeaked": data[7], "dizzyBarYaw": round(data[8], 2), 
                    "dizzyBarPitch": round(data[9], 2), "dizzyBarRoll": round(data[10], 2), 
                    "smoothedTotalEnergy": round(data[11], 2), "frustrationLevel": round(data[12], 2),
                    "isHandTeasing": data[13], "isHandVanishing": data[14], "isHandling": data[15], 
                    "hasExperiencedLift": data[16], "isLowering": data[17], "hasLanded": data[18], "isDizzy": data[19]
                }
                await self.server.send_message(self.channels["events"], timestamp_nanos, json.dumps(doc).encode("utf8"))

            elif msg_id == 131 and len(payload) == struct.calcsize(FMT_COGNITIVE):
                data = struct.unpack(FMT_COGNITIVE, payload)
                await self.server.send_message(self.channels["cognition"], timestamp_nanos, json.dumps({"systemMode": data[0], "robotMood": data[1]}).encode("utf8"))

            elif msg_id == 121 and len(payload) == struct.calcsize(FMT_CONTROL_DEBUG):
                data = struct.unpack(FMT_CONTROL_DEBUG, payload)
                await self.server.send_message(self.channels["control_debug"], timestamp_nanos, json.dumps({"targetHeading": round(data[0], 2), "headingError": round(data[1], 2)}).encode("utf8"))

            elif msg_id == 136 and len(payload) == struct.calcsize(FMT_PERCEPTION):
                data = struct.unpack(FMT_PERCEPTION, payload)
                doc = {
                    "distanceDelta": round(data[0], 3), "totalRawEnergy": round(data[1], 3), 
                    "rawYawEnergy": round(data[2], 3), "rawPitchEnergy": round(data[3], 3), 
                    "rawRollEnergy": round(data[4], 3), "currentGForce": round(data[5], 3)
                }
                await self.server.send_message(self.channels["perception"], timestamp_nanos, json.dumps(doc).encode("utf8"))

            elif msg_id == 101 and len(payload) == struct.calcsize(FMT_PHYSICS):
                data = struct.unpack(FMT_PHYSICS, payload)
                doc = {
                    "yaw": round(data[0], 2), "pitch": round(data[1], 2), "roll": round(data[2], 2), 
                    "gForce": round(data[3], 2), "hasCompass": data[4], "compassHeading": round(data[5], 2), 
                    "leftMotorPWM": data[6], "rightMotorPWM": data[7]
                }
                await self.server.send_message(self.channels["physics"], timestamp_nanos, json.dumps(doc).encode("utf8"))
                
        except Exception as e:
            print(f"Error parsing struct ID {msg_id}: {e}")

if __name__ == "__main__":
    bridge = BLEBridge()
    try:
        asyncio.run(bridge.start())
    except KeyboardInterrupt:
        print("\nBridge Terminated.")