from arduino.app_utils import *
import time
import json
import os

STATE_PATH = os.path.join(os.path.dirname(__file__), "pathfinder_state.json")

# Initialize global state for character-by-character transfer
gps_string_list = []
char_index = 0

def linux_started():
    return True

def save_gps(gps_str):
    try:
        # Split the incoming string: "43.589001,N,-79.644104,W,156.20"
        parts = gps_str.split(',')
        
        if len(parts) == 5:
            # Store EACH data field individually into the JSON
            data = {
                "latitude": float(parts[0]),
                "lat_direction": parts[1].strip(),
                "longitude": float(parts[2]),
                "lon_direction": parts[3].strip(),
                "altitude": float(parts[4]),
                "saved_unix": time.time()
            }
            with open(STATE_PATH, "w") as f:
                json.dump(data, f, indent=4)
            return True
        else:
            print("Received malformed GPS string from Bridge")
            return False
            
    except Exception as e:
        print(f"save_gps error: {e}")
        return False

def load_gps():
    global gps_string_list, char_index
    try:
        if not os.path.exists(STATE_PATH):
            gps_string_list = list("No Data\0") 
            return "No Data"
            
        with open(STATE_PATH, "r") as f:
            data = json.load(f)
        
        # Reconstruct into a clean CSV format for easy C++ parsing
        if "latitude" in data:
            g_str = f"{data['latitude']},{data['lat_direction']},{data['longitude']},{data['lon_direction']},{data['altitude']}"
        else:
            g_str = "No Data"
        
        # Break the string into a list of characters for the C++ Bridge
        gps_string_list = list(g_str)
        gps_string_list.append('\0') # Add null terminator for C++
        
        char_index = 0 # Reset the pointer
        return g_str
        
    except Exception as e:
        print(f"load_gps error: {e}")
        gps_string_list = list("err\0")
        return "error"

def send_gps_char():
    global char_index, gps_string_list
    if char_index < len(gps_string_list):
        char = gps_string_list[char_index]
        char_index += 1
        return char
    return '\0'

def verify_gps():
    try:
        if not os.path.exists(STATE_PATH):
            return "No Data"
        with open(STATE_PATH, "r") as f:
            data = json.load(f)
        if "latitude" in data:
            return f"{data['latitude']}{data['lat_direction']}, {data['longitude']}{data['lon_direction']}"
        return "No Data"
    except:
        return "error"

def reset_iteration():
    global char_index
    char_index = 0

# Register RPC calls
Bridge.provide("linux_started", linux_started)
Bridge.provide("save_gps", save_gps)
Bridge.provide("load_gps", load_gps)
Bridge.provide("send_gps_char", send_gps_char)
Bridge.provide("verify_gps", verify_gps)
Bridge.provide("reset_iteration", reset_iteration)

def loop():
    time.sleep(0.25)

App.run(user_loop=loop)
