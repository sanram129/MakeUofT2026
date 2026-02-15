#include <Wire.h>
#include "DFRobot_GNSSAndRTC.h"
#include <Arduino_RouterBridge.h>

// Initialize GPS (using I2C communication)
#define I2C_COMMUNICATION 
DFRobot_GNSSAndRTC_I2C gnss(&Wire, MODULE_I2C_ADDRESS);

// Button (Must be on A1 to prevent clashing with I2C SCL on A5)
const int buttonPin = A1;
int lastButtonState = HIGH;

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

void setup() {
  // Use Monitor for Uno Q App Lab console output
  Monitor.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  Bridge.begin();
  waitForPython();

  // Start GPS
  if (!gnss.begin()) {
    Monitor.println("ERROR: No GNSS module detected.");
    while (1) { delay(100); }
  }
  gnss.enablePower();
  gnss.setGnss(gnss.eGPS_BeiDou_GLONASS);
  Monitor.println("GNSS Initialized. Waiting for satellite lock...");

  // ==========================================
  // BOOT RECOVERY (Character-by-Character)
  // ==========================================
  String recoveredGPS = "";
  
  // Call the Python function to prepare the GPS string list
  Bridge.call("load_gps"); 

  String inputStr = ""; 
  bool endReached = false;

  while (!endReached) {
    // Call the Python function that yields one character at a time
    RpcCall new_input_call = Bridge.call("send_gps_char");
    
    if (new_input_call.result(inputStr)) {
      if (inputStr == "" || inputStr.charAt(0) == '\0') {
        endReached = true;
      } else {
        char c = inputStr.charAt(0);
        recoveredGPS += c;
        Monitor.print("Got char: ");
        Monitor.println(c);
      }
    }
    delay(10); // Give the Bridge a tiny bit of breathing room
  }
  
  Monitor.print("\nFinal Recovered GPS: ");
  Monitor.println(recoveredGPS);
  
  // Tell Python to reset the character index for the next load
  Bridge.call("reset_iteration");
  Monitor.println("-----------------------------------\n");
}

void loop() {
  Bridge.update();

  int currentButtonState = digitalRead(buttonPin);

  // When button is pressed (transitions from HIGH to LOW)
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    
    // 1. Get GPS Data
    DFRobot_GNSSAndRTC::sLonLat_t lat = gnss.getLat();
    DFRobot_GNSSAndRTC::sLonLat_t lon = gnss.getLon();
    double alt = gnss.getAlt();
    uint8_t satellites = gnss.getNumSatUsed();

    // Only attempt to save if the GPS has actual data from space
    if (satellites > 0) {
      
      // 2. Format Data into a clean, comma-separated list for Python to parse
      // Format: "Latitude,LatDir,Longitude,LonDir,Altitude"
      String gpsStr = String(lat.latitudeDegree, 6) + "," + 
                      String((char)lat.latDirection) + "," + 
                      String(lon.lonitudeDegree, 6) + "," + 
                      String((char)lon.lonDirection) + "," + 
                      String(alt, 2);
      
      // 3. Send to Python via Bridge
      bool ok = false;
      RpcCall c = Bridge.call("save_gps", gpsStr);
      
      if (c.result(ok) && ok) {
        Monitor.print("=> Sent to Linux: ");
        Monitor.println(gpsStr);

        // Verify immediate load to ensure Python processed the JSON correctly
        String verifyStr = "";
        RpcCall v = Bridge.call("verify_gps");
        if (v.result(verifyStr)) {
          Monitor.print("=> Verified JSON: ");
          Monitor.println(verifyStr);
        }
        Monitor.println("-----------------------------------");
      } else {
        Monitor.println("=> Save FAILED via Bridge");
      }
    } else {
      Monitor.println("=> Cannot save: Waiting for Satellite Lock... (Go Outside!)");
    }
    
    // Debounce delay
    delay(200); 
  }

  lastButtonState = currentButtonState;
  delay(10);
}
