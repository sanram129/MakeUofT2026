# MakeUofT2026
Our GPS-based navigation device made for MakeUofT 2026.
# Polaris — GPS + Compass “Home Locator” (MakeUofT Hackathon)

A handheld navigation device that lets you **set a Home waypoint** and then **guides you back** using **live GPS position** + **magnetometer heading**. The UI behaves like a car compass: it shows a **direction arrow** and **cardinal direction text (N/NE/E/…)** in real time.

## Demo (What it does)
- Press **Set Home** → stores your current latitude/longitude as the “Home” waypoint
- As you walk/drive away, the screen updates with:
  - **Distance to Home**
  - **Bearing to Home**
  - **Your current heading + cardinal direction**
  - **An arrow pointing toward Home**

## Key Features
- **Real-time compass UI** (arrow + cardinal direction label)
- **Bearing + distance-to-home** calculations (return-to-home navigation)
- **Sensor fusion-ish smoothing** (basic filtering to reduce jitter)
- **Persistent Home waypoint storage** using a **Linux file-backed JSON store**
  - Home survives restarts/power cycles

## Tech Stack
- **MCU/Firmware:** Arduino (C/C++)
- **Host/Backend:** Python on Linux (JSON persistence)
- **Sensors:** GPS module + magnetometer (e.g., Adafruit LIS2MDL)
- **Display:** ILI9486 TFT shield (DIYables)

---

## System Overview
**Data flow (high-level):**
1. **Arduino** reads GPS + magnetometer and updates the TFT UI
2. **Python (Linux host)** handles **save/load** of the Home waypoint in a JSON file
3. Arduino and Python communicate over **Serial (UART over USB)**

