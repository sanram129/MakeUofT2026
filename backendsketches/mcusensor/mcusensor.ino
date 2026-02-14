#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <Arduino_RouterBridge.h>
#include <math.h>

// Initialize Compass
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);

// Button
const int buttonPin = A1;
int lastButtonState = HIGH;

static float headingDegFromEvent(const sensors_event_t &event) {
  // heading = atan2(Y, X) in degrees, mapped to [0..360)
  float heading = atan2f(event.magnetic.y, event.magnetic.x) * 180.0f / 3.1415926f;
  if (heading < 0) heading += 360.0f;
  return heading;
}

static bool waitForPython(uint32_t timeoutMs = 10000) {
  bool started = false;
  uint32_t t0 = millis();

  // Give Linux/Python a moment (common UNO Q gotcha)
  delay(500);

  while (!started && (millis() - t0) < timeoutMs) {
    RpcCall c = Bridge.call("linux_started");
    c.result(started);     // started becomes true if Python answered
    if (!started) delay(200);
  }
  return started;
}

void setup() {
  Monitor.begin(115200);

  pinMode(buttonPin, INPUT_PULLUP);

  Monitor.println("\n--- Linux Storage Compass Test (UNO Q) ---");

  // Start Bridge
  Bridge.begin();

  bool pythonOK = waitForPython();
  if (!pythonOK) {
    Monitor.println("WARNING: Python not ready. (App python/main.py must be running)");
  } else {
    Monitor.println("Python ready.");
  }

  // Start the Compass
  if (!mag.begin()) {
    Monitor.println("Ooops, no LIS2MDL compass detected ... Check wiring!");
    while (1) { delay(100); }
  }

  // BOOT RECOVERY (load from Linux)
  float recoveredHeading = -1.0f;
  RpcCall r = Bridge.call("load_heading");
  if (r.result(recoveredHeading) && recoveredHeading >= 0.0f) {
    Monitor.print("BOOT RECOVERY: Last saved heading was ");
    Monitor.print(recoveredHeading, 2);
    Monitor.println(" degrees.");
  } else {
    Monitor.println("BOOT RECOVERY: No saved heading yet.");
  }

  Monitor.println("-----------------------------------");
  Monitor.println("Rotate compass, then press button on A1 to SAVE heading to Linux.");
}

void loop() {
  Bridge.update();  // good practice on UNO Q

  int currentButtonState = digitalRead(buttonPin);

  // Detect falling edge (HIGH -> LOW)
  if (lastButtonState == HIGH && currentButtonState == LOW) {

    // Read compass
    sensors_event_t event;
    mag.getEvent(&event);
    float currentHeading = headingDegFromEvent(event);

    // Save to Linux
    bool ok = false;
    RpcCall c = Bridge.call("save_heading", currentHeading);
    if (c.result(ok) && ok) {
      Monitor.println("\n=> BUTTON PRESSED!");
      Monitor.print("=> Saved new heading to Linux: ");
      Monitor.print(currentHeading, 2);
      Monitor.println(" degrees.");

      // Verify load immediately
      float verify = -1.0f;
      RpcCall v = Bridge.call("load_heading");
      if (v.result(verify)) {
        Monitor.print("=> RETRIEVED FROM LINUX: ");
        Monitor.print(verify, 2);
        Monitor.println(" degrees.");
      }
      Monitor.println("TEST: reboot the board/app and you should see it on boot.");
      Monitor.println("-----------------------------------");
    } else {
      Monitor.println("\n=> Save FAILED (is python/main.py running?)");
      Monitor.print("Error code: ");
      Monitor.println(c.getErrorCode());
      Monitor.print("Error msg: ");
      Monitor.println(c.getErrorMessage());
    }

    delay(200); // debounce
  }

  lastButtonState = currentButtonState;
  delay(10);
}
