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
FMT_SYSTEM_HEALTH = "<IIHBB"    # 🚨 CHANGED: Added two 'B's for the CPU loads!
FMT_NETWORK_LINK = "<bb"
FMT_SENSOR_STATE = "<fHh"
FMT_EVENT_STATE = "<8?5f7?"
FMT_COGNITIVE = "<BB"         # 🚨 REVERTED: Now expecting exactly two 1-byte integers for mood and mode each!
FMT_CONTROL_DEBUG = "<ff?"    # 🚨 CHANGED: Expanded to 9 bytes (Added '?' for the usePIDDrive bool)
FMT_PERCEPTION = "<6f"
FMT_PHYSICS = "<ffff?fhh"

# Create dictionaries to translate the raw bytes into human-readable text
MODE_MAP = {
    0: "BOOTING", 1: "MANUAL_OVERRIDE", 2: "NORMAL_DRIVING", 
    3: "OBSTACLE_AVOIDANCE", 4: "MAINTAIN_DISTANCE", 
    5: "COMPASS_LOCK", 6: "DIZZY", 7: "DEEP_SLEEP", 
    8: "AUTOTUNE", 9: "DIAGNOSTICS"
}

MOOD_MAP = {
    0: "HAPPY", 1: "SAD", 2: "ANGRY", 3: "SCARED", 4: "CURIOUS", 5: "SLEEPY", 6: "GROGGY"
}

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
            
            elif msg_id == 130:
                if len(payload) == struct.calcsize(FMT_SYSTEM_HEALTH):
                    data = struct.unpack(FMT_SYSTEM_HEALTH, payload)
                    doc = {
                        "loopTimeUs": data[0], "freeHeap": data[1], "hardwareBitmask": data[2],
                        "cpu0Load": data[3], "cpu1Load": data[4]
                    }
                    await self.server.send_message(self.channels["system_health"], timestamp_nanos, json.dumps(doc).encode("utf8"))
                else:
                    print(f"⚠️ Size Mismatch on ID 130 (Health)! Python expects {struct.calcsize(FMT_SYSTEM_HEALTH)} bytes, ESP32 sent {len(payload)} bytes.")

            elif msg_id == 132:
                if len(payload) == struct.calcsize(FMT_NETWORK_LINK):
                    data = struct.unpack(FMT_NETWORK_LINK, payload)
                    await self.server.send_message(self.channels["network_link"], timestamp_nanos, json.dumps({"wifiRSSI": data[0], "bleRSSI": data[1]}).encode("utf8"))
                else:
                    print(f"⚠️ Size Mismatch on ID 132 (Network)!")

            elif msg_id == 110:
                if len(payload) == struct.calcsize(FMT_SENSOR_STATE):
                    data = struct.unpack(FMT_SENSOR_STATE, payload)
                    doc = {"distanceCM": round(data[0], 2), "batteryVoltageMV": data[1], "currentDrawMA": data[2]}
                    await self.server.send_message(self.channels["sensors"], timestamp_nanos, json.dumps(doc).encode("utf8"))
                else:
                    print(f"⚠️ Size Mismatch on ID 110 (Sensors)!")

            elif msg_id == 135:
                if len(payload) == struct.calcsize(FMT_EVENT_STATE):
                    data = struct.unpack(FMT_EVENT_STATE, payload)
                    # 🚨 RESTORED: The full dictionary mapping is back!
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
                else:
                    print(f"⚠️ Size Mismatch on ID 135 (Events)!")

            elif msg_id == 131:
                if len(payload) == struct.calcsize(FMT_COGNITIVE):
                    data = struct.unpack(FMT_COGNITIVE, payload)
                    
                    # Safely map the integer to the string (or return UNKNOWN if missing)
                    mode_str = MODE_MAP.get(data[0], f"UNKNOWN_MODE({data[0]})")
                    mood_str = MOOD_MAP.get(data[1], f"UNKNOWN_MOOD({data[1]})")
                    
                    doc = {"systemMode": mode_str, "robotMood": mood_str}
                    await self.server.send_message(self.channels["cognition"], timestamp_nanos, json.dumps(doc).encode("utf8"))
                else:
                    print(f"⚠️ Size Mismatch on ID 131 (Cognitive)! Python expects {struct.calcsize(FMT_COGNITIVE)} bytes, ESP32 sent {len(payload)} bytes.")

            elif msg_id == 121:
                if len(payload) == struct.calcsize(FMT_CONTROL_DEBUG):
                    data = struct.unpack(FMT_CONTROL_DEBUG, payload)
                    # 🚨 CHANGED: Added the PID toggle to the dictionary
                    await self.server.send_message(self.channels["control_debug"], timestamp_nanos, json.dumps({"targetHeading": round(data[0], 2), "headingError": round(data[1], 2), "pidEnabled": data[2]}).encode("utf8"))
                else:
                    print(f"⚠️ Size Mismatch on ID 121 (Control Debug)! Python expects {struct.calcsize(FMT_CONTROL_DEBUG)} bytes, ESP32 sent {len(payload)} bytes.")

            elif msg_id == 136:
                if len(payload) == struct.calcsize(FMT_PERCEPTION):
                    data = struct.unpack(FMT_PERCEPTION, payload)
                    doc = {"distanceDelta": round(data[0], 3), "totalRawEnergy": round(data[1], 3), "rawYawEnergy": round(data[2], 3), "rawPitchEnergy": round(data[3], 3), "rawRollEnergy": round(data[4], 3), "currentGForce": round(data[5], 3)}
                    await self.server.send_message(self.channels["perception"], timestamp_nanos, json.dumps(doc).encode("utf8"))
                else:
                    print(f"⚠️ Size Mismatch on ID 136 (Perception)!")

            elif msg_id == 101:
                if len(payload) == struct.calcsize(FMT_PHYSICS):
                    data = struct.unpack(FMT_PHYSICS, payload)
                    doc = {"yaw": round(data[0], 2), "pitch": round(data[1], 2), "roll": round(data[2], 2), "gForce": round(data[3], 2), "hasCompass": data[4], "compassHeading": round(data[5], 2), "leftMotorPWM": data[6], "rightMotorPWM": data[7]}
                    await self.server.send_message(self.channels["physics"], timestamp_nanos, json.dumps(doc).encode("utf8"))
                else:
                    print(f"⚠️ Size Mismatch on ID 101 (Physics)!")
                
        except Exception as e:
            print(f"Error parsing struct ID {msg_id}: {e}")

if __name__ == "__main__":
    bridge = BLEBridge()
    try:
        asyncio.run(bridge.start())
    except KeyboardInterrupt:
        print("\nBridge Terminated.")