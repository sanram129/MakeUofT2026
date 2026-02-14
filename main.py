from arduino.app_utils import *
import time
import json
import os

STATE_PATH = os.path.join(os.path.dirname(__file__), "pathfinder_state.json")

# Initialize global state
home_position_list = []
i = 0

def linux_started():
    return True

def save_heading(heading_str):
    try:
        val = float(heading_str)
        data = {"heading_deg": val, "saved_unix": time.time()}
        with open(STATE_PATH, "w") as f:
            json.dump(data, f)
        return True
    except Exception as e:
        print(f"save_heading error: {e}")
        return False

def load_heading():
    global home_position_list, i
    try:
        if not os.path.exists(STATE_PATH):
            home_position_list = list("-1.0\0") # Default fallback
            return "-1.0"
            
        with open(STATE_PATH, "r") as f:
            data = json.load(f)
        
        # Convert to string safely
        h_str = str(data.get("heading_deg", "-1.0"))
        
        # FIX: list(h_str) turns "12.3" into ['1', '2', '.', '3']
        home_position_list = list(h_str)
        home_position_list.append('\0') # Add null terminator for C++
        
        i = 0 # Reset the pointer
        print(f"Loaded heading: {h_str}")
        return h_str
    except Exception as e:
        print(f"load_heading error: {e}")
        home_position_list = list("err\0")
        return "error"

def send_one_char_heading():
    global i
    # SAFETY CHECK: Only try to access the list if 'i' is within bounds
    if i < len(home_position_list):
        char = home_position_list[i]
        i += 1
        return char
    return '\0' # Return null if the Arduino asks for more than we have

def reset_iteration():
    global i
    i = 0

# Registering methods with the names the Arduino expects
Bridge.provide("linux_started", linux_started)
Bridge.provide("save_heading", save_heading)
Bridge.provide("load_heading", load_heading)
Bridge.provide("send_heading", send_one_char_heading)
Bridge.provide("reset_iteration", reset_iteration)

def loop():
    time.sleep(0.25)

App.run(user_loop=loop)
