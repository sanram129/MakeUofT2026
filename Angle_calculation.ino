#include <DIYables_TFT_Shield.h>
#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include "DFRobot_GNSSAndRTC.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// ================= TFT UI =================
DIYables_TFT_ILI9486_Shield TFT;

// Colors
#define WHITE   DIYables_TFT::colorRGB(255, 255, 255)
#define BLACK   DIYables_TFT::colorRGB(0, 0, 0)
#define MAGENTA DIYables_TFT::colorRGB(255, 0, 255)
#define GREEN   DIYables_TFT::colorRGB(0, 170, 0)
#define RED     DIYables_TFT::colorRGB(200, 0, 0)
#define GRAY    DIYables_TFT::colorRGB(230, 230, 230)

static const int STATUS_H = 28;
static const int BOTTOM_H = 38;
static const int MARGIN   = 8;

float prevAngle = NAN;

// ================= BACKEND (moved into UI sketch) =================
#define I2C_COMMUNICATION
DFRobot_GNSSAndRTC_I2C gnss(&Wire, MODULE_I2C_ADDRESS);
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);

const int buttonPin = A5;
int lastButtonState = HIGH;

// Global variables to hold the retrieved JSON target ("home")
float targetLat = 0.0;
float targetLon = 0.0;
bool hasTarget = false;

static float wrap360(float x) {
  while (x < 0.0f) x += 360.0f;
  while (x >= 360.0f) x -=360.0f;
  return x;
}

// ---------- Bridge wait ----------
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

// Convert 360 Degree Angle to Cardinal String (AS IN YOUR BACKEND)
String getCardinalDirection(float heading) {
  if (heading >= 337.5 || heading < 22.5)  return "N";
  if (heading >= 22.5  && heading < 67.5)  return "NW";
  if (heading >= 67.5  && heading < 112.5) return "W";
  if (heading >= 112.5 && heading < 157.5) return "SW";
  if (heading >= 157.5 && heading < 202.5) return "S";
  if (heading >= 202.5 && heading < 247.5) return "SE";
  if (heading >= 247.5 && heading < 292.5) return "E";
  if (heading >= 292.5 && heading < 337.5) return "NE";
  return "?";
}

// Distance in meters (Haversine) (AS IN YOUR BACKEND)
float calculateDistance(float currLat, float currLon, float destLat, float destLon) {
  float R = 6371000.0;

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
  return R * c;
}

// Bearing (AS IN YOUR BACKEND)
float calculateBearing(float currLat, float currLon, float destLat, float destLon) {
  float lat1 = currLat * PI / 180.0;
  float lon1 = currLon * PI / 180.0;
  float lat2 = destLat * PI / 180.0;
  float lon2 = destLon * PI / 180.0;

  float dLon = lon2 - lon1;
  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

  float bearing = atan2(y, x) * 180.0 / PI;
  if (bearing < 0) bearing += 360.0;
  return bearing;
}

// Retrieve target ("home") from Linux JSON, char-by-char (AS IN YOUR BACKEND)
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
      return;
    }
  }

  hasTarget = false;
}

// ================= Status Bar =================
void drawStatusBarFrame() {
  int W = TFT.width();
  TFT.fillRect(0, 0, W, STATUS_H, GRAY);
  TFT.drawLine(0, STATUS_H - 1, W, STATUS_H - 1, BLACK);
}

void updateStatusBar(bool gpsLock, int sats, float batteryV) {
  int W = TFT.width();

  TFT.setTextSize(2);

  // Left: GPS
  TFT.setCursor(MARGIN, 6);
  TFT.setTextColor(gpsLock ? GREEN : RED, GRAY);
  TFT.print("GPS: ");
  TFT.print(gpsLock ? "LOCK" : "--  ");

  // Middle: SAT
  TFT.setCursor(W / 2 - 40, 6);
  TFT.setTextColor(BLACK, GRAY);
  TFT.print("SAT:");
  TFT.print(sats);
  TFT.print("  ");

  // Right: BAT (kept — you can replace batteryV later if you have a real reading)
  TFT.setCursor(W - 140, 6);
  TFT.setTextColor(BLACK, GRAY);
  TFT.print("BAT:");
  TFT.print(batteryV, 1);
  TFT.print("V ");
}

// ================= Bottom Bar =================
void drawBottomBarFrame() {
  int W = TFT.width();
  int H = TFT.height();
  int y0 = H - BOTTOM_H;
  TFT.fillRect(0, y0, W, BOTTOM_H, GRAY);
  TFT.drawLine(0, y0, W, y0, BLACK);
}

void updateBottomLeftDistance(float distM) {
  int H = TFT.height();
  int y0 = H - BOTTOM_H;

  TFT.setTextSize(2);
  TFT.setTextColor(BLACK, GRAY);
  TFT.setCursor(MARGIN, y0 + 10);

  char buf[24];
  snprintf(buf, sizeof(buf), "HOME: %6.1f m", distM);
  TFT.print(buf);
}

void updateBottomRightDateTime(int yy, int mm, int dd, int hh, int mi, int ss) {
  int W = TFT.width();
  int H = TFT.height();
  int y0 = H - BOTTOM_H;

  TFT.setTextSize(2);
  TFT.setTextColor(BLACK, GRAY);

  char buf[32];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d  %04d-%02d-%02d", hh, mi, ss, yy, mm, dd);

  int len = (int)strlen(buf);
  int approxCharW = 6 * 2;
  int textW = len * approxCharW;

  int x = W - MARGIN - textW;
  if (x < 0) x = 0;

  TFT.setCursor(x, y0 + 10);
  TFT.print(buf);
}

// ================= Arrow Drawing =================
void drawThickLine(int x0, int y0, int x1, int y1, int thickness, uint16_t color) {
  for (int i = -thickness / 2; i <= thickness / 2; i++) {
    TFT.drawLine(x0 + i, y0, x1 + i, y1, color);
    TFT.drawLine(x0, y0 + i, x1, y1 + i, color);
  }
}

void drawArrow(float angleDeg, uint16_t color) {
  int W = TFT.width();
  int H = TFT.height();

  int cx = W / 2;
  int cy = (STATUS_H + (H - BOTTOM_H)) / 2;

  int shaftLen = 70;
  int shaftLenBack = 10;
  int thickness = 6;

  int headLen = 22;
  int headW   = 18;

  float th = angleDeg * (3.1415926f / 180.0f);

  float dx = cosf(th);
  float dy = sinf(th);

  float px = -dy;
  float py = dx;

  int xTip = cx + (int)(dx * shaftLen);
  int yTip = cy + (int)(dy * shaftLen);

  int xShaftEnd = cx + (int)(dx * (shaftLen - headLen));
  int yShaftEnd = cy + (int)(dy * (shaftLen - headLen));

  int xShaftStart = cx - (int)(dx * shaftLenBack);
  int yShaftStart = cy - (int)(dy * shaftLenBack);

  drawThickLine(xShaftStart, yShaftStart, xShaftEnd, yShaftEnd, thickness, color);

  int xBaseC = xShaftEnd;
  int yBaseC = yShaftEnd;

  int xL = xBaseC + (int)(px * (headW / 2));
  int yL = yBaseC + (int)(py * (headW / 2));
  int xR = xBaseC - (int)(px * (headW / 2));
  int yR = yBaseC - (int)(py * (headW / 2));

  TFT.fillTriangle(xTip, yTip, xL, yL, xR, yR, color);
}

// ================= Demo “Clock” (unchanged) =================
struct SoftClock {
  int yy=2026, mm=2, dd=14;
  int hh=12, mi=0, ss=0;
  uint32_t lastMs=0;

  static bool isLeap(int y){ return (y%400==0) || (y%4==0 && y%100!=0); }
  static int daysInMonth(int y,int m){
    static const int d[12]={31,28,31,30,31,30,31,31,30,31,30,31};
    if(m==2) return d[m-1] + (isLeap(y)?1:0);
    return d[m-1];
  }
  void tick() {
    uint32_t now = millis();
    if (lastMs == 0) lastMs = now;
    while (now - lastMs >= 1000) {
      lastMs += 1000;
      ss++;
      if (ss >= 60) { ss=0; mi++; }
      if (mi >= 60) { mi=0; hh++; }
      if (hh >= 24) { hh=0; dd++; }
      int dim = daysInMonth(yy, mm);
      if (dd > dim) { dd=1; mm++; }
      if (mm > 12) { mm=1; yy++; }
    }
  }
} clockSim;

// ================= Info Panel (new, small text block) =================
static void drawInfoPanelFrame() {
  int x = MARGIN;
  int y = STATUS_H + MARGIN;
  int w = 220;
  int h = 120;
  TFT.drawRect(x, y, w, h, BLACK);
}

static void updateInfoPanel(float currLat, float currLon, float altM,
                            float headingDeg, const String &currDir,
                            bool haveTargetData,
                            float angleToTarget, const String &targetDir,
                            const String &distanceStr) {
  int x = MARGIN + 1;
  int y = STATUS_H + MARGIN + 1;
  int w = 220 - 2;
  int h = 120 - 2;

  TFT.fillRect(x, y, w, h, WHITE);

  TFT.setTextSize(2);
  TFT.setTextColor(BLACK, WHITE);

  int cy = y + 4;

  char buf[48];

  snprintf(buf, sizeof(buf), "LAT: %.6f", currLat);
  TFT.setCursor(x + 6, cy); TFT.print(buf);
  cy += 18;

  snprintf(buf, sizeof(buf), "LON: %.6f", currLon);
  TFT.setCursor(x + 6, cy); TFT.print(buf);
  cy += 18;

  snprintf(buf, sizeof(buf), "ALT: %.1f m", altM);
  TFT.setCursor(x + 6, cy); TFT.print(buf);
  cy += 18;

  snprintf(buf, sizeof(buf), "HDG: %.1f %s", headingDeg, currDir.c_str());
  TFT.setCursor(x + 6, cy); TFT.print(buf);
  cy += 18;

  if (haveTargetData) {
    snprintf(buf, sizeof(buf), "BRG: %.1f %s", angleToTarget, targetDir.c_str());
    TFT.setCursor(x + 6, cy); TFT.print(buf);
    cy += 18;

    TFT.setCursor(x + 6, cy);
    TFT.print("DST: ");
    TFT.print(distanceStr);
  } else {
    TFT.setCursor(x + 6, cy);
    TFT.print("BRG: --");
    cy += 18;
    TFT.setCursor(x + 6, cy);
    TFT.print("DST: --");
  }
}

// ================= Setup / Loop =================
float demoVel  = 0.08f; // kept but no longer drives UI distance

void setup() {
  Monitor.begin(9600);

  // Bridge (needed to save/load home GPS JSON exactly like your backend)
  Bridge.begin();
  waitForPython();
  pinMode(buttonPin, INPUT_PULLUP);

  // Sensors init (same as backend)
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

  // TFT init (your UI)
  TFT.begin();
  TFT.setRotation(1);
  TFT.fillScreen(WHITE);

  drawStatusBarFrame();
  drawBottomBarFrame();
  drawInfoPanelFrame();

  // Initial paint
  updateStatusBar(false, 0, 8.7f);
  updateBottomLeftDistance(0.0f);
  updateBottomRightDateTime(clockSim.yy, clockSim.mm, clockSim.dd,
                            clockSim.hh, clockSim.mi, clockSim.ss);

  // Optional center dot
  int cx = TFT.width() / 2;
  int cy = (STATUS_H + (TFT.height() - BOTTOM_H)) / 2;
  TFT.fillCircle(cx, cy, 3, BLACK);
}

void loop() {
  Bridge.update();

  static float angle = 0.0f;       // screen angle used by drawArrow()
  static bool haveNav = false;

  // Live values (computed same as backend)
  static float currLat = 0.0f, currLon = 0.0f, altM = 0.0f;
  static float currentHeading = 0.0f;
  static String currentDirStr = "?";

  static float angleToTarget = 0.0f;
  static String targetDirStr = "?";
  static float distanceInMeters = -1.0f;
  static String distanceStr = "--";

  static uint8_t satellites = 0;
  static bool gpsLock = false;

  // ---- Button press logic (same as backend) ----
  int currentButtonState = digitalRead(buttonPin);

  if (lastButtonState == HIGH && currentButtonState == LOW) {
    if (satellites > 0) {
      // Save CURRENT position to JSON through python (same format as backend)
      DFRobot_GNSSAndRTC::sLonLat_t lat = gnss.getLat();
      DFRobot_GNSSAndRTC::sLonLat_t lon = gnss.getLon();
      double alt = gnss.getAlt();

      String gpsStr = String(lat.latitudeDegree, 6) + "," +
                      String((char)lat.latDirection) + "," +
                      String(lon.lonitudeDegree, 6) + "," +
                      String((char)lon.lonDirection) + "," +
                      String(alt, 2);

      bool ok = false;
      RpcCall c = Bridge.call("save_gps", gpsStr);
      c.result(ok);
    }
    delay(200); // debounce
  }
  lastButtonState = currentButtonState;

  // ---- Status update (real satellites) ----
  static uint32_t lastStatus = 0;
  if (millis() - lastStatus > 500) {
    lastStatus = millis();
    satellites = gnss.getNumSatUsed();
    gpsLock = (satellites > 0);
    updateStatusBar(gpsLock, (int)satellites, 8.7f);
  }

  // ---- Bottom bar clock update (unchanged demo clock) ----
  static uint32_t lastBottom = 0;
  if (millis() - lastBottom > 250) {
    lastBottom = millis();
    clockSim.tick();
    updateBottomRightDateTime(clockSim.yy, clockSim.mm, clockSim.dd,
                              clockSim.hh, clockSim.mi, clockSim.ss);
  }

  // ---- NAV update (same as backend print interval: every 2 seconds) ----
  static uint32_t lastNav = 0;
  if (millis() - lastNav > 2000) {
    lastNav = millis();

    // Current GPS
    DFRobot_GNSSAndRTC::sLonLat_t lat = gnss.getLat();
    DFRobot_GNSSAndRTC::sLonLat_t lon = gnss.getLon();
    altM = (float)gnss.getAlt();
    satellites = gnss.getNumSatUsed();
    gpsLock = (satellites > 0);

    currLat = lat.latitudeDegree;
    if (lat.latDirection == 'S') currLat = -currLat;

    currLon = lon.lonitudeDegree;
    if (lon.lonDirection == 'W') currLon = -currLon;

    // Current heading
    sensors_event_t mag_event;
    mag.getEvent(&mag_event);
    float Pi = 3.14159f;
    currentHeading = (atan2(mag_event.magnetic.y, mag_event.magnetic.x) * 180.0f) / Pi;
    if (currentHeading < 0) currentHeading += 360.0f;

    currentDirStr = getCardinalDirection(currentHeading);

    // Load target (home) from JSON
    retrieveTargetFromLinux();

    // If we have GPS + target, compute nav values
    if (satellites > 0 && hasTarget) {
      float bearingCW = calculateBearing(currLat, currLon, targetLat, targetLon);

      // IMPORTANT:
      // Your codebase + cardinal mapping are using 0=N with CCW increasing.
      // calculateBearing() returns the conventional CW bearing.
      // Convert CW -> CCW so the UI matches your cardinal mapping:
      angleToTarget = wrap360(360.0f - bearingCW);

      targetDirStr = getCardinalDirection(angleToTarget);

      distanceInMeters = calculateDistance(currLat, currLon, targetLat, targetLon);

      if (distanceInMeters >= 1000.0f) {
        distanceStr = String(distanceInMeters / 1000.0f, 2) + " km";
      } else {
        distanceStr = String(distanceInMeters, 0) + " m";
      }

      haveNav = true;

      // Bottom-left uses the real distance (meters)
      updateBottomLeftDistance(distanceInMeters);

      // Arrow: UI drawArrow uses screen-angle (0=RIGHT, 90=DOWN).
      // For CCW-from-North angleToTarget: screenAngle = (270 - angleToTarget) mod 360
      float screenAngle = wrap360(270.0f - angleToTarget);
      angle = screenAngle;

    } else {
      haveNav = false;
      distanceInMeters = -1.0f;
      distanceStr = "--";
    }

    // Update info panel with live backend values
    updateInfoPanel(currLat, currLon, altM,
                    currentHeading, currentDirStr,
                    haveNav,
                    angleToTarget, targetDirStr,
                    distanceStr);
  }

  // ---- Arrow draw/erase ----
  if (!isnan(prevAngle)) drawArrow(prevAngle, WHITE);
  drawArrow(angle, MAGENTA);
  prevAngle = angle;

  // If no nav yet, keep a slow demo spin so screen isn't "dead"
  if (!haveNav) {
    angle += 3.0f;
    if (angle >= 360.0f) angle -= 360.0f;
  }

  delay(25);
}
