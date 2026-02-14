#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <Arduino_RouterBridge.h>
#include <math.h>

// Initialize Compass
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);

// Button
const int buttonPin = A5;
int lastButtonState = HIGH;

static float headingDegFromEvent(const sensors_event_t &event) {
  float heading = atan2f(event.magnetic.y, event.magnetic.x) * 180.0f / 3.1415926f;
  if (heading < 0) heading += 360.0f;
  return heading;
}

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
  Monitor.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  Bridge.begin();
  waitForPython();

  if (!mag.begin()) {
    while (1) { delay(100); }
  }

  // BOOT RECOVERY
  String recoveredHeading = "";
  Bridge.call("load_heading"); // Prepare the list in Python

  String inputStr = ""; 
  bool endReached = false;

  while (!endReached) {
    RpcCall new_input_call = Bridge.call("send_heading");
    
    // Get result as a String to avoid type mismatch
    if (new_input_call.result(inputStr)) {
      if (inputStr == "" || inputStr.charAt(0) == '\0') {
        endReached = true;
      } else {
        char c = inputStr.charAt(0);
        recoveredHeading += c;
        Monitor.print("Got char: ");
        Monitor.println(c);
      }
    }
    delay(10); // Give the Bridge a tiny bit of breathing room
  }
  
  Monitor.print("Final Recovered Heading: ");
  Monitor.println(recoveredHeading);
  Bridge.call("reset_iteration");
}

void loop() {
  Bridge.update();

  int currentButtonState = digitalRead(buttonPin);

  if (lastButtonState == HIGH && currentButtonState == LOW) {
    sensors_event_t event;
    mag.getEvent(&event);
    float currentHeading = headingDegFromEvent(event);

    // FIX: Send heading as a STRING to prevent binary corruption
    String headingStr = String(currentHeading, 2); 
    
    bool ok = false;
    RpcCall c = Bridge.call("save_heading", headingStr);
    
    if (c.result(ok) && ok) {
      Monitor.print("=> Saved: ");
      Monitor.println(headingStr);

      // Verify immediate load
      String verifyStr = "";
      RpcCall v = Bridge.call("load_heading");
      if (v.result(verifyStr)) {
        Monitor.print("=> Verified: ");
        Monitor.println(verifyStr.toFloat());
      }
      Monitor.println("-----------------------------------");
    } else {
      Monitor.println("=> Save FAILED");
    }
    delay(200); 
  }

  lastButtonState = currentButtonState;
  delay(10);
}
