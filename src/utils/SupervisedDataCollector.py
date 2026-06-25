import asyncio
import struct
import csv
import sys
import os
from bleak import BleakClient
from pynput import keyboard

ROBOT_MAC_ADDRESS = "44:1B:F6:FF:94:19" 
CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

# ALIGNED STRUCT FORMATS
FMT_PHYSICS = "<ffff?f"
FMT_SENSOR = "<f?ffffHh"
FMT_ACTUATOR = "<hh?"

last_yaw = 0.0
reset_requested = False 

robot_state = {
    # --- LAYER 1 INPUTS (Real Time Sensors & Actuators) ---
    "pitch": 0.0, "roll": 0.0, "gForce": 1.0, "yawRate": 0.0, 
    "distanceCM": -1.0, "pressureDeltaPa": 0.0, "pressurePa": 0.0,
    "leftMotorPWM": 0, "rightMotorPWM": 0,
    
    # --- LAYER 1 INPUTS (Future Expansion) ---
    "magHeading": 0.0,
    "encLeftVel": 0.0, "encRightVel": 0.0,
    "cliffFL": 0.0, "cliffFR": 0.0, "cliffBL": 0.0, "cliffBR": 0.0,
    
    # --- LAYER 1 OUTPUTS / LAYER 2 INPUTS (The Latches) ---
    "label_isUpright": 1,
    "label_isUpsideDown": 0,    
    "label_isTippedLeft": 0,
    "label_isTippedRight": 0,
    "label_isNoseUp": 0,        
    "label_isNoseDown": 0,      
    "label_isAbsolutelyStill": 1, 
    "label_isHandling": 0,     
    "label_isFreeFalling": 0,
    "label_isStuck": 0,
    "label_hazardDetected": 0, 
    "label_isBeingTeased": 0,
    "label_isBeingPushed": 0,

    # --- LAYER 2 OUTPUTS (The Final Mode Decision) ---
    "target_mode": 10
}

packet_buffer = bytearray()
pressed_keys = set() 

def render_ui():
    sys.stdout.write("\033[H")
    mode_names = {10:"BRAIN_DEAD", 2:"NORMAL", 3:"OBSTACLE", 4:"DIST_HOLD", 5:"COMPASS", 6:"DIZZY", 11:"VERT_BAL"}
    
    ui = f"""
======================================================================
  MISTER MISCHIEF 2-LAYER AI DATA LABELER
======================================================================
 MODES (Sticky):                  ORIENTATIONS (Sticky/Mutually Exclusive):
 [0] BrainDead  [4] Compass       [U] Upright       [I] Upside Down
 [1] Normal     [5] Dizzy         [Left] Tip Left   [Right] Tip Right
 [2] Obstacle   [6] Vertical      [Up] Nose Up      [Down] Nose Down
 [3] Dist Hold

 ACTION LATCHES:
 [SPACE] isHandling (STICKY)           [BACKSPACE] Wipe CSV & Restart
 
 (MOMENTARY - Hold to Activate, Release to Deactivate!):
 [SHIFT] isAbsolutelyStill        [E] isBeingTeased (Sonar Game)
 [W] hazardDetected (Wall)        [R] isStuck (Motors on, no movement)
 [Q] isBeingPushed                [F] isFreeFalling (Zero G)
----------------------------------------------------------------------
 RAW HARDWARE (AI LAYER 1 INPUTS):
 L_PWM: [{robot_state['leftMotorPWM']:>4}]  R_PWM: [{robot_state['rightMotorPWM']:>4}]  Sonar: [{robot_state['distanceCM']:>6.1f} cm]

 AI TARGETS (LATENTS & MODES):
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
    global robot_state, reset_requested
    if key in pressed_keys: return 
    pressed_keys.add(key)
    
    try:
        if key == keyboard.Key.backspace:
            reset_requested = True
            return

        if hasattr(key, 'char') and key.char is not None:
            c = key.char.lower() 
            
            if c == '0': robot_state["target_mode"] = 10 
            elif c == '1': robot_state["target_mode"] = 2 
            elif c == '2': robot_state["target_mode"] = 3 
            elif c == '3': robot_state["target_mode"] = 4 
            elif c == '4': robot_state["target_mode"] = 5 
            elif c == '5': robot_state["target_mode"] = 6 
            elif c == '6': robot_state["target_mode"] = 11 

            elif c == 'u': set_orientation("label_isUpright")
            elif c == 'i': set_orientation("label_isUpsideDown") 
            
            elif c == 'w': robot_state["label_hazardDetected"] = 1
            elif c == 'q': robot_state["label_isBeingPushed"] = 1
            elif c == 'e': robot_state["label_isBeingTeased"] = 1
            elif c == 'r': robot_state["label_isStuck"] = 1
            elif c == 'f': robot_state["label_isFreeFalling"] = 1

        if key == keyboard.Key.space: 
            robot_state["label_isHandling"] = int(not robot_state["label_isHandling"])
            robot_state["label_isAbsolutelyStill"] = 0 if robot_state["label_isHandling"] else 1
            
        elif key == keyboard.Key.shift: robot_state["label_isAbsolutelyStill"] = 0 
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
        if hasattr(key, 'char') and key.char is not None:
            c = key.char.lower()
            if c == 'w': robot_state["label_hazardDetected"] = 0
            elif c == 'q': robot_state["label_isBeingPushed"] = 0
            elif c == 'e': robot_state["label_isBeingTeased"] = 0
            elif c == 'r': robot_state["label_isStuck"] = 0
            elif c == 'f': robot_state["label_isFreeFalling"] = 0
            
        if key == keyboard.Key.shift: robot_state["label_isAbsolutelyStill"] = 1 
        render_ui()
    except AttributeError:
        pass

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

def process_packet(msg_id, payload):
    global robot_state, last_yaw
    try:
        if msg_id == 101 and len(payload) == struct.calcsize(FMT_PHYSICS):
            data = struct.unpack(FMT_PHYSICS, payload)
            
            current_yaw = data[0]
            yaw_delta = current_yaw - last_yaw
            if yaw_delta > 180.0: yaw_delta -= 360.0
            elif yaw_delta < -180.0: yaw_delta += 360.0
                
            robot_state["yawRate"] = round(yaw_delta, 3)
            last_yaw = current_yaw
            
            robot_state["pitch"] = round(data[1], 3)
            robot_state["roll"] = round(data[2], 3)
            robot_state["gForce"] = round(data[3], 3)
            
        elif msg_id == 110 and len(payload) == struct.calcsize(FMT_SENSOR):
            data = struct.unpack(FMT_SENSOR, payload)
            robot_state["distanceCM"] = round(data[0], 2)
            robot_state["pressurePa"] = round(data[2], 2)
            robot_state["pressureDeltaPa"] = round(data[4], 2) 

        elif msg_id == 104 and len(payload) == struct.calcsize(FMT_ACTUATOR):
            data = struct.unpack(FMT_ACTUATOR, payload)
            robot_state["leftMotorPWM"] = data[0]
            robot_state["rightMotorPWM"] = data[1]

    except Exception:
        pass

# ==========================================================
# 🚨 THE MSP MULTIWII PACKET DECODER 🚨
# ==========================================================
def notification_handler(sender, data):
    global packet_buffer
    packet_buffer.extend(data)
    
    # Minimum MSP packet is 6 bytes: '$', 'M', '>', Size, ID, Checksum
    while len(packet_buffer) >= 6:
        # ASCII for '$' is 36, 'M' is 77, '>' is 62
        if packet_buffer[0] == 36 and packet_buffer[1] == 77 and packet_buffer[2] == 62:
            
            payload_size = packet_buffer[3]
            msg_id = packet_buffer[4]
            total_packet_size = 6 + payload_size
            
            # Wait until the full packet (including checksum) has arrived over BLE
            if len(packet_buffer) >= total_packet_size:
                payload = packet_buffer[5 : 5 + payload_size]
                
                # --- CHECKSUM VALIDATION (Matches your C++ logic perfectly) ---
                calc_checksum = packet_buffer[3] ^ packet_buffer[4]
                for b in payload:
                    calc_checksum ^= b
                    
                received_checksum = packet_buffer[5 + payload_size]
                
                if calc_checksum == received_checksum:
                    # Valid packet! Send it to the struct unpacker
                    process_packet(msg_id, payload)
                else:
                    # Silent drop for corrupt BLE packets (prevents AI poisoning)
                    pass 
                
                # Slide the window forward
                packet_buffer = packet_buffer[total_packet_size:]
            else:
                # Packet is fragmented across multiple BLE blasts. Wait for more data.
                break 
        else:
            # We are misaligned. Pop 1 byte to slide the search window forward.
            packet_buffer.pop(0)

async def csv_writer_task():
    global reset_requested
    with open('pristine_ai_data.csv', mode='w', newline='') as file:
        writer = csv.DictWriter(file, fieldnames=robot_state.keys())
        writer.writeheader()
        
        os.system('cls' if os.name == 'nt' else 'clear')
        render_ui()
        
        loop_counter = 0 # <--- NEW: The refresh throttle counter
        
        while True:
            # --- THE RESTART LOGIC ---
            if reset_requested:
                file.seek(0)           
                file.truncate()        
                writer.writeheader()   
                reset_requested = False
                sys.stdout.write("\033[H\033[2J") 
                print("\n\n======================================================================")
                print("  [ SYSTEM RESET ] CSV FILE SUCCESSFULLY WIPED AND RESTARTED!")
                print("======================================================================\n")
                await asyncio.sleep(1.5)
                render_ui()
                
            writer.writerow(robot_state)
            file.flush() 
            
            # --- NEW: UI REFRESH DECOUPLING ---
            # 10 loops * 0.01s = UI redraws every 0.1 seconds (10Hz)
            loop_counter += 1
            if loop_counter >= 10:
                render_ui()
                loop_counter = 0
                
            await asyncio.sleep(0.01) # 100Hz Data Logging

async def main():
    async with BleakClient(ROBOT_MAC_ADDRESS) as client:
        await client.start_notify(CHAR_UUID, notification_handler)
        await csv_writer_task()

if __name__ == "__main__":
    try:
        sys.stdout.write('\033[?25l') 
        sys.stdout.flush()
        asyncio.run(main())
    except KeyboardInterrupt:
        sys.stdout.write('\033[?25h')
        sys.stdout.flush()
        print("\n\n======================================================================")
        print(" [SHUTDOWN] Data Collection Stopped Gracefully.")
        print(" [SUCCESS]  pristine_ai_data.csv safely saved to disk.")
        print("======================================================================\n")
        os._exit(0)