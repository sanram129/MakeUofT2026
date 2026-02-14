# python/main.py
# Stores heading on Linux (persistent) and serves it to the MCU via Bridge RPC.

from arduino.app_utils import *
import time
import json
import os

# Save next to this file so it's inside your App folder (persisted on eMMC)
STATE_PATH = os.path.join(os.path.dirname(__file__), "pathfinder_state.json")

def linux_started():
    # Used by the MCU to wait until Python is ready
    return True

def save_heading(heading_deg: float):
    try:
        data = {
            "heading_deg": float(heading_deg),
            "saved_unix": time.time()
        }
        with open(STATE_PATH, "w") as f:
            json.dump(data, f)
        # print(f"Saved heading: {heading_deg:.2f} -> {STATE_PATH}")
        return True
    except Exception as e:
        print("save_heading error:", e)
        return False

def load_heading():
    try:
        if not os.path.exists(STATE_PATH):
            return -1.0  # means "no saved value yet"
        with open(STATE_PATH, "r") as f:
            data = json.load(f)
        return float(data.get("heading_deg", -1.0))
    except Exception as e:
        print("load_heading error:", e)
        return -1.0

Bridge.provide("linux_started", linux_started)
Bridge.provide("save_heading", save_heading)
Bridge.provide("load_heading", load_heading)

def loop():
    # Keep Python alive to serve RPC calls
    time.sleep(0.25)

App.run(user_loop=loop)
