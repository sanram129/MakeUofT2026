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

// Convert 360 Degree Angle to Cardinal String
String getCardinalDirection(float heading) {
  if (heading >= 337.5 || heading < 22.5)  return "N";
  if (heading >= 22.5  && heading < 67.5)  return "NE";
  if (heading >= 67.5  && heading < 112.5) return "E";
  if (heading >= 112.5 && heading < 157.5) return "SE";
  if (heading >= 157.5 && heading < 202.5) return "S";
  if (heading >= 202.5 && heading < 247.5) return "SW";
  if (heading >= 247.5 && heading < 292.5) return "W";
  if (heading >= 292.5 && heading < 337.5) return "NW";
  return "?"; 
}

// NEW: Math function to calculate distance in meters (Haversine Formula)
float calculateDistance(float currLat, float currLon, float destLat, float destLon) {
  float R = 6371000.0; // Radius of Earth in meters
  
  float lat1 = currLat * PI / 180.0;
  float lon1 = currLon * PI / 180.0;
  float lat2 = destLat * PI / 180.0;
  float lon2 = destLon * PI / 180.0;

  float dLat = lat2 - lat1;
  float dLon = lon2 - lon1;

  float a = sin(dLat / 2.0) * sin(dLat / 2.0) +
            cos(lat1) * cos(lat2) *
            sin(dLon / 2.0) * sin(dLon / 2.0);
            
  float c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  
  return R * c; // Returns distance in meters
}

// Math function to calculate the Compass Bearing
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

// Character-by-Character Retrieval Function
void retrieveTargetFromLinux() {
  String recoveredGPS = "";
  Bridge.call("load_gps"); 
  
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
    delay(5); 
  }
  Bridge.call("reset_iteration");

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
    delay(200); 
  }
  lastButtonState = currentButtonState;

  // ----------------------------------------------------
  // CONTINUOUS LOOP LOGIC (Every 2 seconds)
  // ----------------------------------------------------
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 2000) {
    lastUpdate = millis();
    
    retrieveTargetFromLinux();
    
    String currentDirStr = getCardinalDirection(currentHeading);
    
    Monitor.println("--- Navigation Readout ---");
    Monitor.print("Current Heading:   "); 
    Monitor.print(currentHeading, 1); 
    Monitor.print("° ("); Monitor.print(currentDirStr); Monitor.println(")");
    
    if (satellites > 0 && hasTarget) {
      
      // Calculate Angle & Distance
      float angleToTarget = calculateBearing(currLat, currLon, targetLat, targetLon);
      String targetDirStr = getCardinalDirection(angleToTarget);
      
      float distanceInMeters = calculateDistance(currLat, currLon, targetLat, targetLon);
      
      Monitor.print("Target Bearing:    "); 
      Monitor.print(angleToTarget, 1); 
      Monitor.print("° ("); Monitor.print(targetDirStr); Monitor.println(")");
      
      // NEW: Dynamic Distance Printout (Meters vs Kilometers)
      Monitor.print("Distance to Target: "); 
      if (distanceInMeters >= 1000.0) {
        Monitor.print(distanceInMeters / 1000.0, 2); 
        Monitor.println(" km");
      } else {
        Monitor.print(distanceInMeters, 0); 
        Monitor.println(" m");
      }
      
      Monitor.print("Turn Instruction:  "); 
      
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
    Monitor.println("");
  }
  
  delay(10);
}
