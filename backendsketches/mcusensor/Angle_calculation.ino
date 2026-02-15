#include <Wire.h>
#include "DFRobot_GNSSAndRTC.h"
#include <Arduino_RouterBridge.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <math.h>

// Initialize GPS & Compass
#define I2C_COMMUNICATION
DFRobot_GNSSAndRTC_I2C gnss(&Wire, MODULE_I2C_ADDRESS);
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);

const int buttonPin = A5;
int lastButtonState = HIGH;

// Global variables to hold the retrieved JSON target
float targetLat = 0.0;
float targetLon = 0.0;
bool hasTarget = false;

static bool waitForPython(uint32_t timeoutMs = 10000) {
  bool started = false;
  uint32_t t0 = millis();
  delay(500);
  while (!started && (millis() - t0) < timeoutMs) {
    RpcCall c = Bridge.call("linux_started");
    c.result(started);
    if (!started) delay(200);
  }
  return started;
}

// Math function to calculate the Compass Bearing between two geographical points
float calculateBearing(float currLat, float currLon, float destLat, float destLon) {
  float lat1 = currLat * PI / 180.0;
  float lon1 = currLon * PI / 180.0;
  float lat2 = destLat * PI / 180.0;
  float lon2 = destLon * PI / 180.0;

  float dLon = lon2 - lon1;
  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

  float bearing = atan2(y, x) * 180.0 / PI;
  if (bearing < 0) {
    bearing += 360.0;
  }
  return bearing;
}

// Calculate great-circle distance (Haversine) in meters between two lat/lon points
float haversineMeters(float lat1Deg, float lon1Deg, float lat2Deg, float lon2Deg) {
  const float R = 6371000.0f; // Earth radius (m)
  float lat1 = lat1Deg * PI / 180.0f;
  float lon1 = lon1Deg * PI / 180.0f;
  float lat2 = lat2Deg * PI / 180.0f;
  float lon2 = lon2Deg * PI / 180.0f;

  float dLat = lat2 - lat1;
  float dLon = lon2 - lon1;

  float a = sin(dLat / 2) * sin(dLat / 2) +
            cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  float c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}

// Convert a heading (0..360, 0=N, 90=E) to a cardinal string
String headingToCardinalStr(float deg) {
  static const char* dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
  deg = fmodf(deg, 360.0f);
  if (deg < 0) deg += 360.0f;
  int idx = (int)floor((deg + 22.5f) / 45.0f) % 8;
  return String(dirs[idx]);
}

// Character-by-Character Retrieval Function
void retrieveTargetFromLinux() {
  String recoveredGPS = "";
  Bridge.call("load_gps"); // Prepares the char list in Python

  String inputStr = "";
  bool endReached = false;

  while (!endReached) {
    RpcCall new_input_call = Bridge.call("send_gps_char");
    if (new_input_call.result(inputStr)) {
      if (inputStr == "" || inputStr.charAt(0) == '\0') {
        endReached = true;
      } else {
        recoveredGPS += inputStr.charAt(0);
      }
    }
    delay(5); // Prevent overwhelming the Bridge
  }
  Bridge.call("reset_iteration");

  // Parse the reconstructed CSV string (e.g., "43.589,N,-79.644,W,156.2")
  if (recoveredGPS != "No Data" && recoveredGPS != "error" && recoveredGPS != "") {
    int idx1 = recoveredGPS.indexOf(',');
    int idx2 = recoveredGPS.indexOf(',', idx1 + 1);
    int idx3 = recoveredGPS.indexOf(',', idx2 + 1);
    int idx4 = recoveredGPS.indexOf(',', idx3 + 1);

    if (idx1 > 0 && idx4 > 0) {
      float tLat = recoveredGPS.substring(0, idx1).toFloat();
      String latDir = recoveredGPS.substring(idx1 + 1, idx2);

      float tLon = recoveredGPS.substring(idx2 + 1, idx3).toFloat();
      String lonDir = recoveredGPS.substring(idx3 + 1, idx4);

      // Convert S and W to negative floats for accurate mathematical calculation
      if (latDir == "S") tLat = -tLat;
      if (lonDir == "W") tLon = -tLon;

      targetLat = tLat;
      targetLon = tLon;
      hasTarget = true;
    }
  } else {
    hasTarget = false;
  }
}

void setup() {
  Monitor.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  Bridge.begin();
  waitForPython();

  if (!mag.begin()) {
    Monitor.println("ERROR: No LIS2MDL Compass detected.");
    while (1) { delay(100); }
  }

  if (!gnss.begin()) {
    Monitor.println("ERROR: No GNSS module detected.");
    while (1) { delay(100); }
  }
  gnss.enablePower();
  gnss.setGnss(gnss.eGPS_BeiDou_GLONASS);

  Monitor.println("========================================");
  Monitor.println("     Survival Pathway Finder Ready      ");
  Monitor.println("========================================\n");
}

void loop() {
  Bridge.update();

  int currentButtonState = digitalRead(buttonPin);

  // 1. Get Live GPS Data
  DFRobot_GNSSAndRTC::sLonLat_t lat = gnss.getLat();
  DFRobot_GNSSAndRTC::sLonLat_t lon = gnss.getLon();
  double alt = gnss.getAlt();
  uint8_t satellites = gnss.getNumSatUsed();

  // Convert current coordinates to proper math floats (+/-)
  float currLat = lat.latitudeDegree;
  if (lat.latDirection == 'S') currLat = -currLat;

  float currLon = lon.lonitudeDegree;
  if (lon.lonDirection == 'W') currLon = -currLon;

  // 2. Get Live Compass Heading
  sensors_event_t mag_event;
  mag.getEvent(&mag_event);
  float Pi = 3.14159;
  float currentHeading = (atan2(mag_event.magnetic.y, mag_event.magnetic.x) * 180) / Pi;
  if (currentHeading < 0) currentHeading += 360;

  // ----------------------------------------------------
  // BUTTON PRESS LOGIC: Save Location to JSON
  // ----------------------------------------------------
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    if (satellites > 0) {
      String gpsStr = String(lat.latitudeDegree, 6) + "," +
                      String((char)lat.latDirection) + "," +
                      String(lon.lonitudeDegree, 6) + "," +
                      String((char)lon.lonDirection) + "," +
                      String(alt, 2);

      bool ok = false;
      RpcCall c = Bridge.call("save_gps", gpsStr);
      if (c.result(ok) && ok) {
        Monitor.println("\n[SUCCESS] Current Position OVERWRITTEN to memory!");
      }
    } else {
      Monitor.println("\n[WARNING] Cannot save: Waiting for Satellite Lock...");
    }
    delay(200); // Debounce
  }
  lastButtonState = currentButtonState;

  // ----------------------------------------------------
  // CONTINUOUS LOOP LOGIC (Every 1 second)
  // ----------------------------------------------------
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {
    lastUpdate = millis();

    // Continuously pull the target data char-by-char from Python
    retrieveTargetFromLinux();

    Monitor.println("--- Navigation Readout ---");
    Monitor.print("Current Heading:   "); Monitor.print(currentHeading, 1); Monitor.println("°");

    // Encode current heading into a cardinal direction string (N, NE, E, ...)
    String currentDirStr = headingToCardinalStr(currentHeading);

    // Defaults (used when GPS lock / target missing)
    float distanceM = -1.0f;
    float angleToTarget = -1.0f;

    if (satellites > 0 && hasTarget) {
      // Calculate the bearing (0=N, 90=E, clockwise)
      angleToTarget = calculateBearing(currLat, currLon, targetLat, targetLon);

      // Distance to target (meters)
      distanceM = haversineMeters(currLat, currLon, targetLat, targetLon);

      Monitor.print("Target Bearing:    ");
      Monitor.print(angleToTarget, 1); Monitor.println("°");
      Monitor.print("Distance to Turn:  ");

      // Figure out if you need to turn Left or Right to face the target
      float turnAngle = angleToTarget - currentHeading;
      if (turnAngle < -180) turnAngle += 360;
      if (turnAngle > 180) turnAngle -= 360;

      if (abs(turnAngle) < 5.0) {
        Monitor.println("STRAIGHT AHEAD!");
      } else if (turnAngle > 0) {
        Monitor.print("Turn RIGHT "); Monitor.print(abs(turnAngle), 0); Monitor.println("°");
      } else {
        Monitor.print("Turn LEFT "); Monitor.print(abs(turnAngle), 0); Monitor.println("°");
      }
    } else if (satellites == 0) {
      Monitor.println("Target Bearing:    [Awaiting GPS Lock]");
    } else {
      Monitor.println("Target Bearing:    [No Target Saved]");
    }

    // Push the latest nav data to Linux for the TFT UI (dir, distance, angleToTarget)
    // Format: "<DIR>,<DIST_M>,<ANGLE_DEG>"
    String navStr = currentDirStr + "," + String(distanceM, 1) + "," + String(angleToTarget, 1);
    bool okNav = false;
    RpcCall cn = Bridge.call("save_nav", navStr);
    cn.result(okNav);

    Monitor.println("");
  }

  delay(10);
}
