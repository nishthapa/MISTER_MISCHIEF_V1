import asyncio
import json
import time
from foxglove_websocket.server import FoxgloveServer
import websockets

ESP32_IP = "192.168.1.3"  # <--- HEY! DON'T FORGET TO PUT YOUR IP BACK HERE!
ESP32_PORT = 81           

async def main():
    async with FoxgloveServer(
        "0.0.0.0", 
        8765, 
        "Mister Mischief Bridge", 
        supported_encodings=["json"]
    ) as server:
        
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

        # ==================================================
        # THE NEW INFINITE RETRY LOOP
        # ==================================================
        while True:
            # ... top of script remains the same ...
            print(f"Connecting to Mister Mischief at ws://{ESP32_IP}:{ESP32_PORT}...")
            
            # ==================================================
            # YOU MISSED THIS OUTER LOOP!
            # ==================================================
            while True: 
                try:
                    # Ask to connect (Without compression!)
                    async with websockets.connect(f"ws://{ESP32_IP}:{ESP32_PORT}", compression=None) as ws:
                        print("Connected! Relaying telemetry to Foxglove Studio...")
                        
                        # (You successfully got this inner loop!)
                        while True:
                            try:
                                message = await ws.recv()
                                data = json.loads(message)
                                
                                timestamp_nanos = int(time.time() * 1_000_000_000)
                                await server.send_message(chan_id, timestamp_nanos, json.dumps(data).encode("utf8"))
                            except json.JSONDecodeError:
                                pass 
                                
                except Exception as e:
                    print(f"Connection dropped (Robot off or rebooting). Retrying in 2 seconds...")
                
                # Wait 2 seconds before trying again!
                await asyncio.sleep(2)

if __name__ == "__main__":
    asyncio.run(main())