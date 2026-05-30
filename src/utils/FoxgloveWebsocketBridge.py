import asyncio
import json
import time
from foxglove_websocket.server import FoxgloveServer # <--- THE FIX
import websockets

ESP32_IP = "192.168.1.9"  # <-- CHANGE THIS TO MISTER MISCHIEF'S IP
ESP32_PORT = 81           # <-- Matches SystemConfig::WEBSOCKET_PORT

async def main():
    # 1. Start the Foxglove Server on your laptop (Port 8765)
    async with FoxgloveServer(
        "0.0.0.0", 
        8765, 
        "Mister Mischief Bridge", 
        supported_encodings=["json"] # <--- Added explicit JSON support
    ) as server:
        
        # 2. Tell Foxglove exactly what data to expect in our JSON
        chan_id = await server.add_channel(
            {
                "topic": "telemetry",
                "encoding": "json",
                "schemaName": "Telemetry",
                "schema": json.dumps({
                    "type": "object",
                    "properties": {
                        "yaw": {"type": "number"},
                        "pitch": {"type": "number"},
                        "roll": {"type": "number"},
                        "sonar": {"type": "number"},
                        "mode": {"type": "string"},
                        "brain": {"type": "boolean"}
                    }
                }),
            }
        )

        print(f"Connecting to Mister Mischief at ws://{ESP32_IP}:{ESP32_PORT}...")
        
        # 3. Connect to the ESP32's raw WebSocket stream
        try:
            async with websockets.connect(f"ws://{ESP32_IP}:{ESP32_PORT}", compression=None) as ws:
                print("Connected! Relaying telemetry to Foxglove Studio on ws://localhost:8765...")
                
                # 4. Infinite loop to catch JSON and forward it to Foxglove
                while True:
                    try:
                        message = await ws.recv()
                        data = json.loads(message)
                        
                        # Foxglove requires a high-precision timestamp
                        timestamp_nanos = int(time.time() * 1_000_000_000)
                        
                        # Send the data to the Foxglove UI
                        await server.send_message(chan_id, timestamp_nanos, json.dumps(data).encode("utf8"))
                    except json.JSONDecodeError:
                        pass # Ignore malformed packets if the stream glitches
                        
        except Exception as e:
            print(f"Failed to connect to the robot: {e}")

if __name__ == "__main__":
    asyncio.run(main())