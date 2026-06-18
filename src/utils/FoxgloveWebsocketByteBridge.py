import asyncio
import json
import time
import struct
from foxglove_websocket.server import FoxgloveServer
import websockets

ESP32_IP = "192.168.1.4"  # <--- SET TO MISTER MISCHIEF'S IP
ESP32_PORT = 81           

# =========================================================================
# C++ STRUCT BYTE MAPPINGS (Little Endian format)
# https://docs.python.org/3/library/struct.html#format-characters
# =========================================================================
# SystemHealthState: uint32(I), uint32(I), uint16(H), int8(b), int8(b) -> 12 Bytes
FMT_SYSTEM_HEALTH = "<IIHbb" 

# NetworkLinkState: int8(b), int8(b) -> 2 Bytes
FMT_NETWORK_LINK = "<bb"

# SensorState: float(f), uint16(H), int16(h) -> 8 Bytes
FMT_SENSOR_STATE = "<fHh"

# EventState: 8 bools(?), 5 floats(f), 7 bools(?) -> 35 Bytes
FMT_EVENT_STATE = "<8?5f7?"

# CognitiveState: uint8(B), uint8(B) -> 2 Bytes
FMT_COGNITIVE = "<BB"

# ControlDebugState: float(f), float(f) -> 8 Bytes
FMT_CONTROL_DEBUG = "<ff"

# PerceptionMetrics: 6 floats(6f) -> 24 Bytes
FMT_PERCEPTION = "<6f"

# PhysicsState: 4 floats(4f), 1 bool(?), 1 float(f), 2 int16(2h) -> 25 Bytes
FMT_PHYSICS = "<ffff?fhh"

async def main():
    async with FoxgloveServer("0.0.0.0", 8765, "Mister Mischief Bridge", supported_encodings=["json"]) as server:
        
        # --- 1. REGISTER FOXGLOVE CHANNELS ---
        chan_logs = await server.add_channel({
            "topic": "robot/logs", "encoding": "json", "schemaName": "Logs", "schema": json.dumps({"type": "object"})
        })
        chan_health = await server.add_channel({
            "topic": "robot/system_health", "encoding": "json", "schemaName": "SystemHealth", "schema": json.dumps({"type": "object"})
        })
        chan_network = await server.add_channel({
            "topic": "robot/network_link", "encoding": "json", "schemaName": "NetworkLink", "schema": json.dumps({"type": "object"})
        })
        chan_sensors = await server.add_channel({
            "topic": "robot/sensors", "encoding": "json", "schemaName": "Sensors", "schema": json.dumps({"type": "object"})
        })
        chan_events = await server.add_channel({"topic": "robot/events", "encoding": "json", "schemaName": "EventState", "schema": json.dumps({"type": "object"})
        })
        chan_cognition = await server.add_channel({
            "topic": "robot/cognition", "encoding": "json", "schemaName": "CognitiveState", "schema": json.dumps({"type": "object"})
        })
        chan_debug = await server.add_channel({
            "topic": "robot/control_debug", "encoding": "json", "schemaName": "ControlDebug", "schema": json.dumps({"type": "object"})
        })
        chan_perception = await server.add_channel({
            "topic": "robot/perception", "encoding": "json", "schemaName": "PerceptionMetrics", "schema": json.dumps({"type": "object"})
        })
        chan_physics = await server.add_channel({
            "topic": "robot/physics", "encoding": "json", "schemaName": "PhysicsState", "schema": json.dumps({"type": "object"})
        })

        while True:
            print(f"\nConnecting to Binary Stream at ws://{ESP32_IP}:{ESP32_PORT}...")
            
            while True: 
                try:
                    async with websockets.connect(f"ws://{ESP32_IP}:{ESP32_PORT}", compression=None) as ws:
                        print("Connected! Relaying Binary Telemetry to Foxglove...")
                        
                        while True:
                            message = await ws.recv() # Receives RAW BYTES
                            
                            # ----------------------------------------------------
                            # 2. MULTIWII PACKET PARSER & CHECKSUM VALIDATOR
                            # ----------------------------------------------------
                            # Check Header: '$' (36), 'M' (77), '>' (62)
                            if len(message) >= 6 and message[0] == 36 and message[1] == 77 and message[2] == 62:
                                size = message[3]
                                msg_id = message[4]
                                payload = message[5 : 5 + size]
                                checksum = message[5 + size]
                                
                                # Verify XOR Checksum
                                calc_checksum = size ^ msg_id
                                for b in payload:
                                    calc_checksum ^= b
                                    
                                if calc_checksum != checksum:
                                    print(f"Checksum mismatch on ID {msg_id}! Dropping packet.")
                                    continue
                                
                                # ----------------------------------------------------
                                # 3. ROUTER: UNPACK C++ STRUCTS BASED ON MSG ID
                                # ----------------------------------------------------
                                timestamp_nanos = int(time.time() * 1_000_000_000)
                                
                                # ID 140: TRANSIENT_ALERTS (String)
                                if msg_id == 140:
                                    text = payload.decode('utf-8', errors='ignore')
                                    await server.send_message(chan_logs, timestamp_nanos, json.dumps({"log": text}).encode("utf8"))
                                    print(f"[ROBOT] {text}")
                                
                                # ID 130: SYSTEM_STATUS (SystemHealthState Struct)
                                elif msg_id == 130:
                                    if len(payload) == struct.calcsize(FMT_SYSTEM_HEALTH):
                                        data = struct.unpack(FMT_SYSTEM_HEALTH, payload)
                                        doc = {
                                            "loopTimeUs": data[0],
                                            "freeHeap": data[1],
                                            "hardwareBitmask": data[2],
                                            "wifiRSSI": data[3],
                                            "bleRSSI": data[4]
                                        }
                                        await server.send_message(chan_health, timestamp_nanos, json.dumps(doc).encode("utf8"))

                                # ID 132: NETWORK_LINK (NetworkLinkState Struct)
                                elif msg_id == 132:
                                    if len(payload) == struct.calcsize(FMT_NETWORK_LINK):
                                        data = struct.unpack(FMT_NETWORK_LINK, payload)
                                        doc = {
                                            "wifiRSSI": data[0],
                                            "bleRSSI": data[1]
                                        }
                                        await server.send_message(chan_network, timestamp_nanos, json.dumps(doc).encode("utf8"))

                                # ID 110: DISTANCE_SONAR (SensorState Struct)
                                elif msg_id == 110:
                                    if len(payload) == struct.calcsize(FMT_SENSOR_STATE):
                                        data = struct.unpack(FMT_SENSOR_STATE, payload)
                                        doc = {
                                            "distanceCM": round(data[0], 2),
                                            "batteryVoltageMV": data[1],
                                            "currentDrawMA": data[2]
                                        }
                                        await server.send_message(chan_sensors, timestamp_nanos, json.dumps(doc).encode("utf8"))

                                # ID 135: EVENT_STATE (Semantic Latches & Metrics)
                                elif msg_id == 135:
                                    if len(payload) == struct.calcsize(FMT_EVENT_STATE):
                                        data = struct.unpack(FMT_EVENT_STATE, payload)
                                        doc = {
                                            # 1. Semantic Events
                                            "hazardDetected": data[0],
                                            "teaseConfirmed": data[1],
                                            "targetVanished": data[2],
                                            "dizzyTriggered": data[3],
                                            "dizzyFinished": data[4],
                                            "readyForCompassLock": data[5],
                                            "safelyLanded": data[6],
                                            "frustrationPeaked": data[7],
                                            
                                            # 2. Internal Metrics
                                            "dizzyBarYaw": round(data[8], 2),
                                            "dizzyBarPitch": round(data[9], 2),
                                            "dizzyBarRoll": round(data[10], 2),
                                            "smoothedTotalEnergy": round(data[11], 2),
                                            "frustrationLevel": round(data[12], 2),
                                            
                                            # 3. Internal Latches
                                            "isHandTeasing": data[13],
                                            "isHandVanishing": data[14],
                                            "isHandling": data[15],
                                            "hasExperiencedLift": data[16],
                                            "isLowering": data[17],
                                            "hasLanded": data[18],
                                            "isDizzy": data[19]
                                        }
                                        await server.send_message(chan_events, timestamp_nanos, json.dumps(doc).encode("utf8"))

                                # ID 131: ROBOT_STATE (CognitiveState Struct)
                                elif msg_id == 131:
                                    if len(payload) == struct.calcsize(FMT_COGNITIVE):
                                        data = struct.unpack(FMT_COGNITIVE, payload)
                                        doc = {
                                            "systemMode": data[0], # 0=Booting, 1=Teleop, 2=Normal, etc.
                                            "robotMood": data[1]   # 0=Idle, 1=Curious, 5=Aggressive
                                        }
                                        await server.send_message(chan_cognition, timestamp_nanos, json.dumps(doc).encode("utf8"))

                                # ID 121: PID_DRIVE_DEBUG (ControlDebugState Struct)
                                elif msg_id == 121:
                                    if len(payload) == struct.calcsize(FMT_CONTROL_DEBUG):
                                        data = struct.unpack(FMT_CONTROL_DEBUG, payload)
                                        doc = {
                                            "targetHeading": round(data[0], 2),
                                            "headingError": round(data[1], 2)
                                        }
                                        await server.send_message(chan_debug, timestamp_nanos, json.dumps(doc).encode("utf8"))

                                # ID 136: PERCEPTION_METRICS (Energy & G-Force)
                                elif msg_id == 136:
                                    if len(payload) == struct.calcsize(FMT_PERCEPTION):
                                        data = struct.unpack(FMT_PERCEPTION, payload)
                                        doc = {
                                            "distanceDelta": round(data[0], 3),
                                            "totalRawEnergy": round(data[1], 3),
                                            "rawYawEnergy": round(data[2], 3),
                                            "rawPitchEnergy": round(data[3], 3),
                                            "rawRollEnergy": round(data[4], 3),
                                            "currentGForce": round(data[5], 3)
                                        }
                                        await server.send_message(chan_perception, timestamp_nanos, json.dumps(doc).encode("utf8"))

                                # ID 101: ATTITUDE / PHYSICS (PhysicsState Struct)
                                # Note: Assuming you broadcast the whole PhysicsState under ID 101
                                elif msg_id == 101:
                                    if len(payload) == struct.calcsize(FMT_PHYSICS):
                                        data = struct.unpack(FMT_PHYSICS, payload)
                                        doc = {
                                            "yaw": round(data[0], 2),
                                            "pitch": round(data[1], 2),
                                            "roll": round(data[2], 2),
                                            "gForce": round(data[3], 2),
                                            "hasCompass": data[4],
                                            "compassHeading": round(data[5], 2),
                                            "leftMotorPWM": data[6],
                                            "rightMotorPWM": data[7]
                                        }
                                        await server.send_message(chan_physics, timestamp_nanos, json.dumps(doc).encode("utf8"))

                except Exception as e:
                    print(f"Connection dropped: {e}. Retrying in 2 seconds...")
                
                await asyncio.sleep(2)

if __name__ == "__main__":
    asyncio.run(main())