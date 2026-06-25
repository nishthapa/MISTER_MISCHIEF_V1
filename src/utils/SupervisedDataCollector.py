import asyncio
import struct
import csv
import sys
import os
from bleak import BleakClient
from pynput import keyboard

ROBOT_MAC_ADDRESS = "44:1B:F6:FF:94:19" 
CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

FMT_PHYSICS = "<ffff?h" 
FMT_SENSOR = "<f?ffffHh"

robot_state = {
    # --- REAL TIME SENSORS ---
    "pitch": 0.0, "roll": 0.0, "gForce": 1.0,
    "distanceCM": -1.0, "pressureDeltaPa": 0.0, "pressurePa": 0.0,
    
    # --- FUTURE SENSORS ---
    "magHeading": 0.0,
    "encLeftVel": 0.0, "encRightVel": 0.0,
    "cliffFL": 0.0, "cliffFR": 0.0, "cliffBL": 0.0, "cliffBR": 0.0,
    
    # --- TARGET LABELS ---
    "target_mode": 10,
    "label_isUpright": 1,
    "label_isUpsideDown": 0,    # <--- NEW
    "label_isTippedLeft": 0,
    "label_isTippedRight": 0,
    "label_isNoseUp": 0,        # <--- RENAMED
    "label_isNoseDown": 0,      # <--- RENAMED
    
    "label_isAbsolutelyStill": 1, 
    "label_isHandling": 0,     
    "label_isFreeFalling": 0,
    "label_isStuck": 0,
    "label_hazardDetected": 0, 
    "label_isBeingTeased": 0,
    "label_isBeingPushed": 0
}

packet_buffer = bytearray()
pressed_keys = set() # Tracks held keys to prevent OS key-repeat spam

def render_ui():
    # ANSI Escape sequence to move cursor to top-left without flickering
    sys.stdout.write("\033[H")
    
    mode_names = {10:"BRAIN_DEAD", 2:"NORMAL", 3:"OBSTACLE", 4:"DIST_HOLD", 5:"COMPASS", 6:"DIZZY", 11:"VERT_BAL"}
    
    ui = f"""
    ======================================================================
    MISTER MISCHIEF PERCEPTION DATA LABELER (MOMENTARY SWITCH UI)
    ======================================================================
    MODES (Sticky):                  ORIENTATIONS (Sticky/Mutually Exclusive):
    [0] BrainDead  [4] Compass       [U] Upright       [I] Upside Down
    [1] Normal     [5] Dizzy         [Left] Tip Left   [Right] Tip Right
    [2] Obstacle   [6] Vertical      [Up] Nose Up      [Down] Nose Down
    [3] Dist Hold

    ACTION LATCHES (Hold Key Down to Activate, Release to Deactivate!):
    [SPACE] isHandling (Lifted)      [E] isBeingTeased (Sonar Game)
    [SHIFT] isAbsolutelyStill        [R] isStuck (Motors on, no movement)
    [W] hazardDetected (Wall)        [F] isFreeFalling (Zero G)
    [Q] isBeingPushed 
    ----------------------------------------------------------------------
    LIVE AI TRAINING LABELS:
    MODE:        [{mode_names.get(robot_state['target_mode'], 'UNKNOWN'):<12}] 
    ORIENTATION: [Upright:{robot_state['label_isUpright']} | UpsideDown:{robot_state['label_isUpsideDown']} | NoseUp:{robot_state['label_isNoseUp']} | NoseDown:{robot_state['label_isNoseDown']} | TipL:{robot_state['label_isTippedLeft']} | TipR:{robot_state['label_isTippedRight']}]
    
    ACTIONS:
    isHandling:        [{'ON ' if robot_state['label_isHandling'] else 'OFF'}]    isBeingTeased: [{ 'ON ' if robot_state['label_isBeingTeased'] else 'OFF'}]
    isAbsolutelyStill: [{'ON ' if robot_state['label_isAbsolutelyStill'] else 'OFF'}]    isStuck:       [{ 'ON ' if robot_state['label_isStuck'] else 'OFF'}]
    hazardDetected:    [{'ON ' if robot_state['label_hazardDetected'] else 'OFF'}]    isFreeFalling: [{ 'ON ' if robot_state['label_isFreeFalling'] else 'OFF'}]
    isBeingPushed:     [{'ON ' if robot_state['label_isBeingPushed'] else 'OFF'}]
    ======================================================================
    """
    sys.stdout.write(ui)
    sys.stdout.flush()

def set_orientation(target_label):
    robot_state["label_isUpright"] = 0
    robot_state["label_isUpsideDown"] = 0
    robot_state["label_isTippedLeft"] = 0
    robot_state["label_isTippedRight"] = 0
    robot_state["label_isNoseUp"] = 0
    robot_state["label_isNoseDown"] = 0
    robot_state[target_label] = 1

def on_press(key):
    global robot_state
    if key in pressed_keys: return # Ignore key-repeat
    pressed_keys.add(key)
    
    try:
        # --- MODES (Sticky) ---
        if hasattr(key, 'char'):
            if key.char == '0': robot_state["target_mode"] = 10 
            elif key.char == '1': robot_state["target_mode"] = 2 
            elif key.char == '2': robot_state["target_mode"] = 3 
            elif key.char == '3': robot_state["target_mode"] = 4 
            elif key.char == '4': robot_state["target_mode"] = 5 
            elif key.char == '5': robot_state["target_mode"] = 6 
            elif key.char == '6': robot_state["target_mode"] = 11 

            # --- ORIENTATIONS (Sticky) ---
            elif key.char == 'u': set_orientation("label_isUpright")
            elif key.char == 'i': set_orientation("label_isUpsideDown") # <--- NEW
            
            # --- ACTION LATCHES (Momentary PRESS) ---
            elif key.char == 'w': robot_state["label_hazardDetected"] = 1
            elif key.char == 'q': robot_state["label_isBeingPushed"] = 1
            elif key.char == 'e': robot_state["label_isBeingTeased"] = 1
            elif key.char == 'r': robot_state["label_isStuck"] = 1
            elif key.char == 'f': robot_state["label_isFreeFalling"] = 1

        # --- SPECIAL KEYS ---
        if key == keyboard.Key.space: robot_state["label_isHandling"] = 1
        elif key == keyboard.Key.shift: robot_state["label_isAbsolutelyStill"] = 0 # Holding Shift means moving
        elif key == keyboard.Key.left: set_orientation("label_isTippedLeft")
        elif key == keyboard.Key.right: set_orientation("label_isTippedRight")
        elif key == keyboard.Key.up: set_orientation("label_isNoseUp")
        elif key == keyboard.Key.down: set_orientation("label_isNoseDown")
        
        render_ui()
    except AttributeError:
        pass

def on_release(key):
    global robot_state
    if key in pressed_keys: pressed_keys.remove(key)
    
    try:
        # --- ACTION LATCHES (Momentary RELEASE) ---
        if hasattr(key, 'char'):
            if key.char == 'w': robot_state["label_hazardDetected"] = 0
            elif key.char == 'q': robot_state["label_isBeingPushed"] = 0
            elif key.char == 'e': robot_state["label_isBeingTeased"] = 0
            elif key.char == 'r': robot_state["label_isStuck"] = 0
            elif key.char == 'f': robot_state["label_isFreeFalling"] = 0
            
        if key == keyboard.Key.space: robot_state["label_isHandling"] = 0
        elif key == keyboard.Key.shift: robot_state["label_isAbsolutelyStill"] = 1 # Releasing Shift means still
        
        render_ui()
    except AttributeError:
        pass

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

def process_packet(msg_id, payload):
    global robot_state
    try:
        if msg_id == 101 and len(payload) == struct.calcsize(FMT_PHYSICS):
            data = struct.unpack(FMT_PHYSICS, payload)
            robot_state["pitch"] = round(data[1], 3)
            robot_state["roll"] = round(data[2], 3)
            robot_state["gForce"] = round(data[3], 3)
            
        elif msg_id == 110 and len(payload) == struct.calcsize(FMT_SENSOR):
            data = struct.unpack(FMT_SENSOR, payload)
            robot_state["distanceCM"] = round(data[0], 2)
            robot_state["pressurePa"] = round(data[2], 2)
            robot_state["pressureDeltaPa"] = round(data[4], 2) 
    except Exception:
        pass

def notification_handler(sender, data):
    global packet_buffer
    packet_buffer.extend(data)
    
    while len(packet_buffer) >= 4:
        if packet_buffer[0] == 0xAA and packet_buffer[1] == 0x55:
            length = packet_buffer[2]
            if len(packet_buffer) >= length + 4:
                msg_id = packet_buffer[3]
                payload = packet_buffer[4 : 4 + length - 1] 
                process_packet(msg_id, payload)
                packet_buffer = packet_buffer[length + 4:] 
            else:
                break 
        else:
            packet_buffer.pop(0) 

async def csv_writer_task():
    with open('pristine_ai_data.csv', mode='w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=robot_state.keys())
        writer.writeheader()
        
        # Clear screen once at startup
        os.system('cls' if os.name == 'nt' else 'clear')
        render_ui()
        
        while True:
            writer.writerow(robot_state)
            file.flush() 
            # Changed from 0.1 (10Hz) to 0.01 (100Hz)
            await asyncio.sleep(0.01)
            # await asyncio.sleep(0.1) 

async def main():
    async with BleakClient(ROBOT_MAC_ADDRESS) as client:
        await client.start_notify(CHAR_UUID, notification_handler)
        await csv_writer_task()

if __name__ == "__main__":
    asyncio.run(main())