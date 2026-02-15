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

// ================= BACKEND =================
#define I2C_COMMUNICATION
DFRobot_GNSSAndRTC_I2C gnss(&Wire, MODULE_I2C_ADDRESS);
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);

const int buttonPin = A5;
int lastButtonState = HIGH;

// HOME saved in Linux JSON (loaded via Bridge)
float targetLat = 0.0f;
float targetLon = 0.0f;
float targetAlt = 0.0f;
bool  hasTarget = false;

// ---------- Tunables ----------
static const uint32_t MAG_CAL_MS = 8000;         // rotate device for ~8s on boot
static const float    DIST_ALPHA = 0.20f;        // distance smoothing 0.1..0.3
static const float    COURSE_MIN_MOVE_M = 2.5f;  // must move this much to trust course heading
static const uint32_t COURSE_VALID_MS = 8000;    // course heading stays valid this long after last update

// If your headings are consistently off, tweak these:
static const float MAG_DECLINATION_DEG = 0.0f;   // set to local declination if desired
static const float MAG_HEADING_OFFSET_DEG = 0.0f;

// If axis seems rotated or mirrored, flip these (0/1)
#define MAG_SWAP_XY   0
#define MAG_INVERT_X  0
#define MAG_INVERT_Y  0

// ================= LAYOUT (INFO PANEL + ARROW AREA) =================
// Info panel moved/defined explicitly
static const int INFO_X = MARGIN;
static const int INFO_Y = STATUS_H + MARGIN;
static const int INFO_W = 260;   // wider than before
static const int INFO_H = 170;   // taller than before

// Arrow area lives to the right of the info panel
int ARROW_X=0, ARROW_Y=0, ARROW_W=0, ARROW_H=0;
int ARROW_CX=0, ARROW_CY=0;

// ---------- Helpers ----------
static float wrap360(float x) {
  while (x < 0.0f)    x += 360.0f;
  while (x >= 360.0f) x -= 360.0f;
  return x;
}

static float wrap180(float x) {
  while (x < -180.0f) x += 360.0f;
  while (x > 180.0f)  x -= 360.0f;
  return x;
}

// circular smoothing for angles
static float smoothAngle(float prev, float now, float alpha) {
  float d = wrap180(now - prev);
  return wrap360(prev + alpha * d);
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

// ✅ Standard CW-from-North mapping (0=N, 90=E, 180=S, 270=W)
String getCardinalDirection(float heading) {
  heading = wrap360(heading);
  if (heading >= 337.5f || heading < 22.5f)  return "N";
  if (heading < 67.5f)   return "NE";
  if (heading < 112.5f)  return "E";
  if (heading < 157.5f)  return "SE";
  if (heading < 202.5f)  return "S";
  if (heading < 247.5f)  return "SW";
  if (heading < 292.5f)  return "W";
  if (heading < 337.5f)  return "NW";
  return "?";
}

// Haversine distance (meters)
float calculateDistance(float currLat, float currLon, float destLat, float destLon) {
  float R = 6371000.0f;
  float lat1 = currLat * PI / 180.0f;
  float lon1 = currLon * PI / 180.0f;
  float lat2 = destLat * PI / 180.0f;
  float lon2 = destLon * PI / 180.0f;

  float dLat = lat2 - lat1;
  float dLon = lon2 - lon1;

  float a = sin(dLat / 2.0f) * sin(dLat / 2.0f) +
            cos(lat1) * cos(lat2) *
            sin(dLon / 2.0f) * sin(dLon / 2.0f);

  float c = 2.0f * atan2(sqrt(a), sqrt(1.0f - a));
  return R * c;
}

// Bearing CW-from-North (degrees)
float calculateBearing(float currLat, float currLon, float destLat, float destLon) {
  float lat1 = currLat * PI / 180.0f;
  float lon1 = currLon * PI / 180.0f;
  float lat2 = destLat * PI / 180.0f;
  float lon2 = destLon * PI / 180.0f;

  float dLon = lon2 - lon1;
  float y = sin(dLon) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);

  float bearing = atan2(y, x) * 180.0f / PI;
  if (bearing < 0) bearing += 360.0f;
  return bearing;
}

// ---------- Magnetometer calibration + corrected heading ----------
float magOffX = 0, magOffY = 0;
float magScaleX = 1, magScaleY = 1;

void calibrateMag2D(uint32_t ms = 8000) {
  float minX =  1e9f, maxX = -1e9f;
  float minY =  1e9f, maxY = -1e9f;

  uint32_t t0 = millis();
  sensors_event_t e;

  while (millis() - t0 < ms) {
    mag.getEvent(&e);

    float x = e.magnetic.x;
    float y = e.magnetic.y;

#if MAG_SWAP_XY
    float tmp = x; x = y; y = tmp;
#endif
#if MAG_INVERT_X
    x = -x;
#endif
#if MAG_INVERT_Y
    y = -y;
#endif

    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;

    delay(40);
  }

  magOffX = (maxX + minX) * 0.5f;
  magOffY = (maxY + minY) * 0.5f;

  float rx = (maxX - minX) * 0.5f;
  float ry = (maxY - minY) * 0.5f;
  float avg = (rx + ry) * 0.5f;

  magScaleX = (rx > 1e-6f) ? (avg / rx) : 1.0f;
  magScaleY = (ry > 1e-6f) ? (avg / ry) : 1.0f;
}

// corrected CW-from-North magnetic heading
float getMagHeadingCW() {
  sensors_event_t e;
  mag.getEvent(&e);

  float x = e.magnetic.x;
  float y = e.magnetic.y;

#if MAG_SWAP_XY
  float tmp = x; x = y; y = tmp;
#endif
#if MAG_INVERT_X
  x = -x;
#endif
#if MAG_INVERT_Y
  y = -y;
#endif

  // hard/soft-iron correction
  float mx = (x - magOffX) * magScaleX;
  float my = (y - magOffY) * magScaleY;

  // raw angle CCW from +X
  float raw = atan2(my, mx) * 180.0f / PI;

  // convert to CW-from-North
  float heading = wrap360(90.0f - raw);

  heading = wrap360(heading + MAG_DECLINATION_DEG + MAG_HEADING_OFFSET_DEG);
  return heading;
}

// ---------- GPS course heading ----------
bool  havePrevFix = false;
float prevFixLat = 0, prevFixLon = 0;
uint32_t prevFixMs = 0;

bool  haveCourse = false;
float courseHeading = 0;
uint32_t lastCourseMs = 0;

// update course heading if we've moved enough
void updateCourseHeading(float currLat, float currLon) {
  uint32_t now = millis();

  if (!havePrevFix) {
    havePrevFix = true;
    prevFixLat = currLat;
    prevFixLon = currLon;
    prevFixMs  = now;
    haveCourse = false;
    return;
  }

  float stepDist = calculateDistance(prevFixLat, prevFixLon, currLat, currLon);

  // only accept course if we actually moved (filters jitter)
  if (stepDist >= COURSE_MIN_MOVE_M) {
    float newCourse = calculateBearing(prevFixLat, prevFixLon, currLat, currLon);

    if (!haveCourse) {
      courseHeading = newCourse;
      haveCourse = true;
    } else {
      courseHeading = smoothAngle(courseHeading, newCourse, 0.35f);
    }
    lastCourseMs = now;

    // update baseline ONLY when moved enough
    prevFixLat = currLat;
    prevFixLon = currLon;
    prevFixMs  = now;
  }

  // expire if old
  if (haveCourse && (now - lastCourseMs > COURSE_VALID_MS)) {
    haveCourse = false;
  }
}

// ---------- Linux HOME retrieval ----------
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

      float tAlt = 0.0f;
      if (idx4 + 1 < (int)recoveredGPS.length()) {
        tAlt = recoveredGPS.substring(idx4 + 1).toFloat();
      }

      if (latDir == "S") tLat = -tLat;
      if (lonDir == "W") tLon = -tLon;

      targetLat = tLat;
      targetLon = tLon;
      targetAlt = tAlt;
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

  TFT.setCursor(MARGIN, 6);
  TFT.setTextColor(gpsLock ? GREEN : RED, GRAY);
  TFT.print("GPS: ");
  TFT.print(gpsLock ? "LOCK" : "--  ");

  TFT.setCursor(W / 2 - 40, 6);
  TFT.setTextColor(BLACK, GRAY);
  TFT.print("SAT:");
  TFT.print(sats);
  TFT.print("  ");

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

  TFT.fillRect(0, y0 + 1, 200, BOTTOM_H - 2, GRAY);

  TFT.setTextSize(2);
  TFT.setTextColor(BLACK, GRAY);
  TFT.setCursor(MARGIN, y0 + 10);

  TFT.print("HOME: ");
  if (distM < 0) {
    TFT.print("--");
  } else {
    TFT.print(distM, 1);
    TFT.print(" m");
  }
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

  TFT.fillRect(x - 4, y0 + 1, textW + 10, BOTTOM_H - 2, GRAY);

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

void drawArrowAt(int cx, int cy, float angleDeg, uint16_t color) {
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

// ================= Demo “Clock” =================
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

// ================= Info Panel =================
static void drawInfoPanelFrame() {
  // draw once initially
  TFT.fillRect(INFO_X, INFO_Y, INFO_W, INFO_H, WHITE);
  TFT.drawRect(INFO_X, INFO_Y, INFO_W, INFO_H, BLACK);
}

static void updateInfoPanel(float currLat, float currLon, float altM,
                            float headingUsedDeg, const String &dirUsed, char headingSrc,
                            bool haveHome,
                            float homeLat, float homeLon, float homeAlt,
                            bool haveTargetData,
                            float angleToTarget, const String &targetDir,
                            const String &distanceStr) {

  // Clear ENTIRE panel and redraw border to avoid overlay artifacts
  TFT.fillRect(INFO_X, INFO_Y, INFO_W, INFO_H, WHITE);
  TFT.drawRect(INFO_X, INFO_Y, INFO_W, INFO_H, BLACK);

  TFT.setTextSize(2);
  TFT.setTextColor(BLACK, WHITE);

  int x = INFO_X + 6;
  int cy = INFO_Y + 6;

  // current
  TFT.setCursor(x, cy); TFT.print("LAT: "); TFT.print(currLat, 6); cy += 18;
  TFT.setCursor(x, cy); TFT.print("LON: "); TFT.print(currLon, 6); cy += 18;
  TFT.setCursor(x, cy); TFT.print("ALT: "); TFT.print(altM, 1); TFT.print("m"); cy += 18;

  // home
  if (haveHome) {
    TFT.setCursor(x, cy); TFT.print("HLA: "); TFT.print(homeLat, 6); cy += 18;
    TFT.setCursor(x, cy); TFT.print("HLN: "); TFT.print(homeLon, 6); cy += 18;
  } else {
    TFT.setCursor(x, cy); TFT.print("HLA: --"); cy += 18;
    TFT.setCursor(x, cy); TFT.print("HLN: --"); cy += 18;
  }

  // heading
  TFT.setCursor(x, cy);
  TFT.print("HDG: "); TFT.print(headingUsedDeg, 1); TFT.print(" ");
  TFT.print(dirUsed); TFT.print(" ");
  TFT.print(headingSrc); // 'G' or 'M'
  cy += 18;

  // bearing + distance
  if (haveTargetData) {
    TFT.setCursor(x, cy);
    TFT.print("BRG: "); TFT.print(angleToTarget, 1); TFT.print(" "); TFT.print(targetDir);
    cy += 18;

    TFT.setCursor(x, cy);
    TFT.print("DST: "); TFT.print(distanceStr);
  } else {
    TFT.setCursor(x, cy); TFT.print("BRG: --"); cy += 18;
    TFT.setCursor(x, cy); TFT.print("DST: --");
  }
}

// ================= Setup / Loop =================
float distFilt = -1.0f;

void setup() {
  Monitor.begin(9600);

  Bridge.begin();
  waitForPython();
  pinMode(buttonPin, INPUT_PULLUP);

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

  TFT.begin();
  TFT.setRotation(1);
  TFT.fillScreen(WHITE);

  // Compute arrow area on the RIGHT of the info panel
  ARROW_X = INFO_X + INFO_W + MARGIN;
  ARROW_Y = STATUS_H;
  ARROW_W = TFT.width()  - ARROW_X - MARGIN;
  ARROW_H = TFT.height() - STATUS_H - BOTTOM_H;

  if (ARROW_W < 40) ARROW_W = 40;
  if (ARROW_H < 40) ARROW_H = 40;

  ARROW_CX = ARROW_X + ARROW_W / 2;
  ARROW_CY = ARROW_Y + ARROW_H / 2;

  drawStatusBarFrame();
  drawBottomBarFrame();
  drawInfoPanelFrame();

  updateStatusBar(false, 0, 8.7f);
  updateBottomLeftDistance(-1.0f);
  updateBottomRightDateTime(clockSim.yy, clockSim.mm, clockSim.dd,
                            clockSim.hh, clockSim.mi, clockSim.ss);

  // Clear arrow area + center dot
  TFT.fillRect(ARROW_X, ARROW_Y, ARROW_W, ARROW_H, WHITE);
  TFT.fillCircle(ARROW_CX, ARROW_CY, 3, BLACK);

  // Load HOME if exists
  retrieveTargetFromLinux();

  // One-time calibration on boot
  TFT.setTextSize(2);
  TFT.setTextColor(BLACK, WHITE);
  TFT.setCursor(MARGIN, STATUS_H + MARGIN + 160);
  TFT.print("Calibrating MAG... rotate");
  calibrateMag2D(MAG_CAL_MS);
  TFT.fillRect(MARGIN, STATUS_H + MARGIN + 160, 320, 18, WHITE);
}

void loop() {
  Bridge.update();

  static float arrowAngleScreen = 0.0f; // 0=right, 90=down
  static bool haveNav = false;

  static float currLat = 0.0f, currLon = 0.0f, altM = 0.0f;

  static float headingUsed = 0.0f;   // CW-from-N
  static char  headingSrc = 'M';     // 'M' mag or 'G' gps course
  static String dirUsed = "?";

  static float angleToTarget = 0.0f; // CW-from-N
  static String targetDirStr = "?";
  static float distanceRaw = -1.0f;
  static String distanceStr = "--";

  static uint8_t satellites = 0;
  static bool gpsLock = false;

  // ---- Button press: Save HOME to JSON ----
  int currentButtonState = digitalRead(buttonPin);
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    satellites = gnss.getNumSatUsed();
    if (satellites > 0) {
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

      retrieveTargetFromLinux(); // update screen right away
    }
    delay(200);
  }
  lastButtonState = currentButtonState;

  // ---- Status update ----
  static uint32_t lastStatus = 0;
  if (millis() - lastStatus > 500) {
    lastStatus = millis();
    satellites = gnss.getNumSatUsed();
    gpsLock = (satellites > 0);
    updateStatusBar(gpsLock, (int)satellites, 8.7f);
  }

  // ---- Clock update ----
  static uint32_t lastBottom = 0;
  if (millis() - lastBottom > 250) {
    lastBottom = millis();
    clockSim.tick();
    updateBottomRightDateTime(clockSim.yy, clockSim.mm, clockSim.dd,
                              clockSim.hh, clockSim.mi, clockSim.ss);
  }

  // ---- NAV update (every 2 seconds) ----
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

    // Update course heading from GPS motion (if moved enough)
    if (gpsLock) {
      updateCourseHeading(currLat, currLon);
    } else {
      haveCourse = false;
    }

    // Magnetometer heading (corrected)
    float headingMag = getMagHeadingCW();

    // Choose heading source:
    // - If course heading is valid (moving), use it
    // - Else use magnetometer (stationary)
    if (haveCourse) {
      headingUsed = courseHeading;
      headingSrc = 'G';
    } else {
      headingUsed = headingMag;
      headingSrc = 'M';
    }
    dirUsed = getCardinalDirection(headingUsed);

    // Load HOME from JSON
    retrieveTargetFromLinux();

    if (gpsLock && hasTarget) {
      angleToTarget = calculateBearing(currLat, currLon, targetLat, targetLon);
      targetDirStr = getCardinalDirection(angleToTarget);

      distanceRaw = calculateDistance(currLat, currLon, targetLat, targetLon);

      // distance smoothing
      if (distFilt < 0) distFilt = distanceRaw;
      distFilt = distFilt + DIST_ALPHA * (distanceRaw - distFilt);

      float distShow = distFilt;

      if (distShow >= 1000.0f) {
        distanceStr = String(distShow / 1000.0f, 2) + " km";
      } else {
        distanceStr = String(distShow, 0) + " m";
      }

      haveNav = true;
      updateBottomLeftDistance(distShow);

      // Arrow: bearing CW-from-N -> screen angle (0=right, 90=down)
      // N(0) should point UP => screen 270
      arrowAngleScreen = wrap360(angleToTarget + 270.0f);

    } else {
      haveNav = false;
      distanceRaw = -1.0f;
      distFilt = -1.0f;
      distanceStr = "--";
      updateBottomLeftDistance(-1.0f);
    }

    updateInfoPanel(currLat, currLon, altM,
                    headingUsed, dirUsed, headingSrc,
                    hasTarget, targetLat, targetLon, targetAlt,
                    haveNav, angleToTarget, targetDirStr,
                    distanceStr);
  }

  // ---- Arrow draw (clean: redraw only inside arrow area) ----
  TFT.fillRect(ARROW_X, ARROW_Y, ARROW_W, ARROW_H, WHITE);
  TFT.fillCircle(ARROW_CX, ARROW_CY, 3, BLACK);
  drawArrowAt(ARROW_CX, ARROW_CY, arrowAngleScreen, MAGENTA);

  // If no nav yet, keep arrow alive
  if (!haveNav) {
    arrowAngleScreen = wrap360(arrowAngleScreen + 3.0f);
  }

  delay(25);
}
