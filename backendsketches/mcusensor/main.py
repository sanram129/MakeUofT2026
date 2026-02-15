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


def _load_state():
    if not os.path.exists(STATE_PATH):
        return {}
    try:
        with open(STATE_PATH, "r") as f:
            return json.load(f) or {}
    except Exception:
        return {}


def _save_state(data: dict) -> bool:
    try:
        with open(STATE_PATH, "w") as f:
            json.dump(data, f, indent=4)
        return True
    except Exception:
        return False


def save_gps(gps_str):
    try:
        # Split the incoming string: "43.589001,N,-79.644104,W,156.20"
        parts = gps_str.split(',')

        if len(parts) == 5:
            # Preserve existing fields (e.g., nav_*)
            data = _load_state()

            data.update({
                "latitude": float(parts[0]),
                "lat_direction": parts[1].strip(),
                "longitude": float(parts[2]),
                "lon_direction": parts[3].strip(),
                "altitude": float(parts[4]),
                "saved_unix": time.time()
            })
            return _save_state(data)

        print("Received malformed GPS string from Bridge")
        return False

    except Exception as e:
        print(f"save_gps error: {e}")
        return False


def load_gps():
    global gps_string_list, char_index
    try:
        data = _load_state()

        # Reconstruct into a clean CSV format for easy C++ parsing
        if "latitude" in data:
            g_str = f"{data['latitude']},{data['lat_direction']},{data['longitude']},{data['lon_direction']},{data['altitude']}"
        else:
            g_str = "No Data"

        # Break the string into a list of characters for the C++ Bridge
        gps_string_list = list(g_str)
        gps_string_list.append('\0')  # null terminator for C++
        char_index = 0
        return g_str

    except Exception as e:
        print(f"load_gps error: {e}")
        gps_string_list = list("err\0")
        return "error"


def send_gps_char():
    global char_index, gps_string_list
    if char_index < len(gps_string_list):
        ch = gps_string_list[char_index]
        char_index += 1
        return ch
    return '\0'


def verify_gps():
    try:
        data = _load_state()
        if "latitude" in data:
            return f"{data['latitude']}{data['lat_direction']}, {data['longitude']}{data['lon_direction']}"
        return "No Data"
    except Exception:
        return "error"


def reset_iteration():
    global char_index
    char_index = 0


# ---------------- NAV DATA (dir, distance, angleToTarget) ----------------
def save_nav(nav_str):
    """
    nav_str format: "<DIR>,<DIST_M>,<ANGLE_DEG>"
    example: "NW,123.4,45.0"
    """
    try:
        parts = str(nav_str).strip().split(",")
        if len(parts) != 3:
            return False

        d = parts[0].strip().upper()
        dist_m = float(parts[1].strip())
        ang = float(parts[2].strip())

        data = _load_state()
        data["nav_dir"] = d
        data["nav_dist_m"] = dist_m
        data["nav_angle_deg"] = ang
        data["nav_saved_unix"] = time.time()

        return _save_state(data)

    except Exception as e:
        print(f"save_nav error: {e}")
        return False


def load_nav():
    """Returns "<DIR>,<DIST_M>,<ANGLE_DEG>" or "No Data"."""
    try:
        data = _load_state()
        if "nav_dir" not in data:
            return "No Data"
        return f"{data.get('nav_dir','--')},{data.get('nav_dist_m',-1)},{data.get('nav_angle_deg',-1)}"
    except Exception as e:
        print(f"load_nav error: {e}")
        return "error"


# Register RPC calls
Bridge.provide("linux_started", linux_started)
Bridge.provide("save_gps", save_gps)
Bridge.provide("load_gps", load_gps)
Bridge.provide("send_gps_char", send_gps_char)
Bridge.provide("verify_gps", verify_gps)
Bridge.provide("reset_iteration", reset_iteration)

Bridge.provide("save_nav", save_nav)
Bridge.provide("load_nav", load_nav)


def loop():
    time.sleep(0.25)


App.run(user_loop=loop)
