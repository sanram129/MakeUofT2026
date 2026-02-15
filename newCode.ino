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
static const float DIST_ALPHA = 0.20f; // distance smoothing 0.1..0.3

// ======= SPEED CONTROLS =======
static const uint32_t GPS_UPDATE_MS       = 1500; // GPS + bearing update rate
static const uint32_t HEADING_UPDATE_MS   = 40;   // compass read rate (25 Hz)
static const uint32_t HEADING_TEXT_MS     = 80;   // heading line UI rate (~12.5 Hz)
static const uint32_t PANEL_UPDATE_MS     = 600;  // other panel lines update rate
static const uint32_t ARROW_UPDATE_MS     = 25;   // arrow redraw rate (~40 FPS)

// Heading smoothing (higher = more responsive, less smooth)
static const float HEADING_ALPHA = 0.35f;

// ======= Compass correction =======
static const float MAG_DECLINATION_DEG    = 0.0f;     // optional (magnetic->true)
static const float MAG_HEADING_OFFSET_DEG = -65.0f;   // your +65° correction

// If compass seems rotated/mirrored, change these (0/1)
#define MAG_SWAP_XY   0
#define MAG_INVERT_X  0
#define MAG_INVERT_Y  0

// ---------- TIMEZONE ----------
static const int TIMEZONE_OFFSET_MIN = 0; // EST=-300, EDT=-240

// ================= LAYOUT =================
static const int INFO_X = MARGIN;
static const int INFO_Y = STATUS_H + MARGIN;
static const int INFO_W = 260;
static const int INFO_H = 190;

int ARROW_X=0, ARROW_Y=0, ARROW_W=0, ARROW_H=0;
int ARROW_CX=0, ARROW_CY=0;

// Arrow clear radius (small square instead of wiping the whole right side)
static const int ARROW_CLEAR_R = 115;

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
static float smoothAngle(float prev, float now, float alpha) {
  float d = wrap180(now - prev);
  return wrap360(prev + alpha * d);
}

// ---- Date helpers (for timezone adjustment + ticking) ----
static bool isLeapYear(int y) { return (y%400==0) || (y%4==0 && y%100!=0); }
static int daysInMonth(int y, int m) {
  static const int d[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (m == 2) return d[m-1] + (isLeapYear(y) ? 1 : 0);
  return d[m-1];
}
static void incOneDay(int &yy, int &mm, int &dd) {
  dd++;
  int dim = daysInMonth(yy, mm);
  if (dd > dim) { dd = 1; mm++; }
  if (mm > 12) { mm = 1; yy++; }
}
static void decOneDay(int &yy, int &mm, int &dd) {
  dd--;
  if (dd < 1) {
    mm--;
    if (mm < 1) { mm = 12; yy--; }
    dd = daysInMonth(yy, mm);
  }
}
static void addMinutesToDateTime(int &yy, int &mm, int &dd, int &hh, int &mi, int deltaMin) {
  int total = hh * 60 + mi + deltaMin;
  while (total < 0)    { total += 1440; decOneDay(yy, mm, dd); }
  while (total >= 1440){ total -= 1440; incOneDay(yy, mm, dd); }
  hh = total / 60;
  mi = total % 60;
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

// Cardinal directions
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
  return wrap360(bearing);
}

// ---------- Compass heading (NO calibration) ----------
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

  float raw = atan2f(y, x) * 180.0f / PI; // CCW from +X
  float heading = wrap360(90.0f - raw);   // CW-from-N

  heading = wrap360(heading + MAG_DECLINATION_DEG + MAG_HEADING_OFFSET_DEG);
  return heading;
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
  if (distM < 0) TFT.print("--");
  else { TFT.print(distM, 1); TFT.print(" m"); }
}

void updateBottomRightText(const char *buf) {
  int W = TFT.width();
  int H = TFT.height();
  int y0 = H - BOTTOM_H;

  TFT.setTextSize(2);
  TFT.setTextColor(BLACK, GRAY);

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

// ================= GNSS-Synced Clock =================
struct SoftClock {
  int yy=2000, mm=1, dd=1;
  int hh=0, mi=0, ss=0;
  uint32_t lastMs=0;
  bool valid=false;

  void set(int y,int m,int d,int h,int mn,int s) {
    yy=y; mm=m; dd=d; hh=h; mi=mn; ss=s;
    lastMs = millis();
    valid = true;
  }
  void tick() {
    if (!valid) return;
    uint32_t now = millis();
    if (lastMs == 0) lastMs = now;
    while (now - lastMs >= 1000) {
      lastMs += 1000;
      ss++;
      if (ss >= 60) { ss=0; mi++; }
      if (mi >= 60) { mi=0; hh++; }
      if (hh >= 24) { hh=0; incOneDay(yy, mm, dd); }
    }
  }
} clockSim;

static bool syncClockFromGNSS() {
  DFRobot_GNSSAndRTC::sTim_t utc  = gnss.getUTC();
  DFRobot_GNSSAndRTC::sTim_t date = gnss.getDate();

  if (date.year < 2000 || date.year > 2099) return false;
  if (utc.hour > 23 || utc.minute > 59 || utc.second > 59) return false;

  int yy = (int)date.year;
  int mm = (int)date.month;
  int dd = (int)date.date;
  int hh = (int)utc.hour;
  int mi = (int)utc.minute;
  int ss = (int)utc.second;

  if (TIMEZONE_OFFSET_MIN != 0) addMinutesToDateTime(yy, mm, dd, hh, mi, TIMEZONE_OFFSET_MIN);
  clockSim.set(yy, mm, dd, hh, mi, ss);
  return true;
}

// ================= FAST INFO PANEL (only redraw changed lines) =================
static const int INFO_LINE_H = 18;
static const int INFO_LINES  = 9;   // number of lines we draw
char prevLine[INFO_LINES][42];      // cache previous printed text

static void drawInfoPanelFrame() {
  TFT.fillRect(INFO_X, INFO_Y, INFO_W, INFO_H, WHITE);
  TFT.drawRect(INFO_X, INFO_Y, INFO_W, INFO_H, BLACK);
  for (int i=0;i<INFO_LINES;i++) prevLine[i][0] = '\0';
}

static void infoPrintLine(int idx, const char *text) {
  if (idx < 0 || idx >= INFO_LINES) return;

  if (strncmp(prevLine[idx], text, sizeof(prevLine[idx])) == 0) return;
  strncpy(prevLine[idx], text, sizeof(prevLine[idx]) - 1);
  prevLine[idx][sizeof(prevLine[idx]) - 1] = '\0';

  int x = INFO_X + 1;
  int y = INFO_Y + 1 + idx * INFO_LINE_H;

  TFT.fillRect(x, y, INFO_W - 2, INFO_LINE_H, WHITE);

  TFT.setTextSize(2);
  TFT.setTextColor(BLACK, WHITE);
  TFT.setCursor(INFO_X + 6, y + 2);
  TFT.print(text);
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

  // Arrow area on RIGHT of the info panel
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
  updateBottomRightText("--:--:--  ---- -- --");

  TFT.fillRect(ARROW_X, ARROW_Y, ARROW_W, ARROW_H, WHITE);
  TFT.fillCircle(ARROW_CX, ARROW_CY, 3, BLACK);

  // Load HOME once on boot
  retrieveTargetFromLinux();
}

void loop() {
  Bridge.update();

  // State
  static bool gpsLock = false;
  static uint8_t satellites = 0;

  static float currLat = 0.0f, currLon = 0.0f, altM = 0.0f;

  static float headingDeg = 0.0f;
  static bool  haveHeading = false;

  static float bearingDeg = 0.0f;
  static bool  haveNav = false;

  static float relDegSigned = 0.0f;
  static float arrowAngleScreen = 270.0f;

  static String distanceStr = "--";

  // ----- Button press: Save HOME -----
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

      // Refresh HOME immediately after save
      retrieveTargetFromLinux();
    }
    delay(200);
  }
  lastButtonState = currentButtonState;

  // ----- Status update -----
  static uint32_t lastStatus = 0;
  if (millis() - lastStatus > 500) {
    lastStatus = millis();
    satellites = gnss.getNumSatUsed();
    gpsLock = (satellites > 0);
    updateStatusBar(gpsLock, (int)satellites, 8.7f);
  }

  // ----- Clock update -----
  static uint32_t lastClock = 0;
  if (millis() - lastClock > 250) {
    lastClock = millis();
    clockSim.tick();
    if (clockSim.valid) {
      char buf[32];
      snprintf(buf, sizeof(buf), "%02d:%02d:%02d  %04d-%02d-%02d",
               clockSim.hh, clockSim.mi, clockSim.ss, clockSim.yy, clockSim.mm, clockSim.dd);
      updateBottomRightText(buf);
    } else {
      updateBottomRightText("--:--:--  ---- -- --");
    }
  }

  // ----- HEADING update (fast) -----
  static uint32_t lastHeading = 0;
  if (millis() - lastHeading > HEADING_UPDATE_MS) {
    lastHeading = millis();
    float h = getMagHeadingCW();
    if (!haveHeading) { headingDeg = h; haveHeading = true; }
    else              { headingDeg = smoothAngle(headingDeg, h, HEADING_ALPHA); }
  }

  // ----- Heading line UI update (fast, independent of panel) -----
  static uint32_t lastHeadingText = 0;
  if (millis() - lastHeadingText > HEADING_TEXT_MS) {
    lastHeadingText = millis();
    char line[64];
    snprintf(line, sizeof(line), "HDG: %.1f %s", headingDeg, getCardinalDirection(headingDeg).c_str());
    infoPrintLine(5, line); // line 5 dedicated to heading
  }

  // ----- GPS + bearing update (slower) -----
  static uint32_t lastGPS = 0;
  if (millis() - lastGPS > GPS_UPDATE_MS) {
    lastGPS = millis();

    DFRobot_GNSSAndRTC::sLonLat_t lat = gnss.getLat();
    DFRobot_GNSSAndRTC::sLonLat_t lon = gnss.getLon();
    altM = (float)gnss.getAlt();
    satellites = gnss.getNumSatUsed();
    gpsLock = (satellites > 0);

    currLat = lat.latitudeDegree;
    if (lat.latDirection == 'S') currLat = -currLat;

    currLon = lon.lonitudeDegree;
    if (lon.lonDirection == 'W') currLon = -currLon;

    if (gpsLock) syncClockFromGNSS();

    // IMPORTANT: do NOT reload HOME every GPS cycle (Bridge calls stall UI)
    // retrieveTargetFromLinux();

    if (gpsLock && hasTarget) {
      bearingDeg = calculateBearing(currLat, currLon, targetLat, targetLon);

      float distanceRaw = calculateDistance(currLat, currLon, targetLat, targetLon);
      if (distFilt < 0) distFilt = distanceRaw;
      distFilt = distFilt + DIST_ALPHA * (distanceRaw - distFilt);

      float distShow = distFilt;
      if (distShow >= 1000.0f) distanceStr = String(distShow / 1000.0f, 2) + " km";
      else                      distanceStr = String(distShow, 0) + " m";

      haveNav = true;
      updateBottomLeftDistance(distShow);
    } else {
      haveNav = false;
      distFilt = -1.0f;
      distanceStr = "--";
      updateBottomLeftDistance(-1.0f);
    }
  }

  // ----- Compute REL (bearing - heading) whenever we have both -----
  if (haveNav && haveHeading) {
    relDegSigned = wrap180(bearingDeg - headingDeg);
    float rel360 = wrap360(relDegSigned);
    arrowAngleScreen = wrap360(rel360 + 270.0f);
  }

  // ----- PANEL update (slow; everything EXCEPT heading line) -----
  static uint32_t lastPanel = 0;
  if (millis() - lastPanel > PANEL_UPDATE_MS) {
    lastPanel = millis();

    char line[64];

    snprintf(line, sizeof(line), "LAT: %.6f", currLat); infoPrintLine(0, line);
    snprintf(line, sizeof(line), "LON: %.6f", currLon); infoPrintLine(1, line);
    snprintf(line, sizeof(line), "ALT: %.1fm", altM);   infoPrintLine(2, line);

    if (hasTarget) {
      snprintf(line, sizeof(line), "HLA: %.6f", targetLat); infoPrintLine(3, line);
      snprintf(line, sizeof(line), "HLN: %.6f", targetLon); infoPrintLine(4, line);
    } else {
      infoPrintLine(3, "HLA: --");
      infoPrintLine(4, "HLN: --");
    }

    // line 5 is heading; updated separately above

    if (haveNav) {
      snprintf(line, sizeof(line), "BRG: %.1f %s", bearingDeg, getCardinalDirection(bearingDeg).c_str());
      infoPrintLine(6, line);

      snprintf(line, sizeof(line), "REL: %.1f deg", relDegSigned);
      infoPrintLine(7, line);

      snprintf(line, sizeof(line), "DST: %s", distanceStr.c_str());
      infoPrintLine(8, line);
    } else {
      infoPrintLine(6, "BRG: --");
      infoPrintLine(7, "REL: --");
      infoPrintLine(8, "DST: --");
    }
  }

  // ----- ARROW redraw (small clear area, fixed FPS) -----
  static uint32_t lastArrow = 0;
  if (millis() - lastArrow > ARROW_UPDATE_MS) {
    lastArrow = millis();

    int x0 = ARROW_CX - ARROW_CLEAR_R;
    int y0 = ARROW_CY - ARROW_CLEAR_R;
    int w  = 2 * ARROW_CLEAR_R;
    int h  = 2 * ARROW_CLEAR_R;

    if (x0 < ARROW_X) x0 = ARROW_X;
    if (y0 < ARROW_Y) y0 = ARROW_Y;
    if (x0 + w > ARROW_X + ARROW_W) w = (ARROW_X + ARROW_W) - x0;
    if (y0 + h > ARROW_Y + ARROW_H) h = (ARROW_Y + ARROW_H) - y0;

    TFT.fillRect(x0, y0, w, h, WHITE);
    TFT.fillCircle(ARROW_CX, ARROW_CY, 3, BLACK);

    float ang = haveNav ? arrowAngleScreen : wrap360(arrowAngleScreen + 3.0f);
    if (!haveNav) arrowAngleScreen = ang;

    drawArrowAt(ARROW_CX, ARROW_CY, ang, MAGENTA);
  }

  delay(5);
}
