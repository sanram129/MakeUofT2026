#include <DIYables_TFT_Shield.h>
#include <Arduino_RouterBridge.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

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
bool pythonReady = false;

// ---------------- Bridge Helpers ----------------
static bool waitForPython(uint32_t timeoutMs = 8000) {
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

// Try to get heading in degrees from backend.
// Works with both styles:
//  1) load_heading returns float (simple)
//  2) load_heading returns string
//  3) load_heading + send_heading (char-by-char)
static bool getHeadingDeg(float &outDeg) {
  // (1) float return
  {
    float deg = -1.0f;
    RpcCall c = Bridge.call("load_heading");
    if (c.result(deg)) {
      if (!isnan(deg) && deg >= 0.0f) {
        outDeg = fmodf(deg, 360.0f);
        if (outDeg < 0) outDeg += 360.0f;
        return true;
      }
    }
  }

  // (2) string return directly
  {
    String s = "";
    RpcCall c = Bridge.call("load_heading");
    if (c.result(s)) {
      float deg = s.toFloat();
      if (deg >= 0.0f) {
        outDeg = fmodf(deg, 360.0f);
        if (outDeg < 0) outDeg += 360.0f;
        return true;
      }
    }
  }

  // (3) char-by-char (send_heading/reset_iteration)
  {
    Bridge.call("load_heading"); // prepare internal list on Python side (for that implementation)
    String recovered = "";
    for (int k = 0; k < 24; k++) { // safety cap
      String ch = "";
      RpcCall cc = Bridge.call("send_heading");
      if (!cc.result(ch) || ch.length() == 0 || ch.charAt(0) == '\0') break;
      recovered += ch.charAt(0);
      delay(2);
    }
    Bridge.call("reset_iteration");

    float deg = recovered.toFloat();
    if (deg >= 0.0f) {
      outDeg = fmodf(deg, 360.0f);
      if (outDeg < 0) outDeg += 360.0f;
      return true;
    }
  }

  return false;
}

// ---------------- Cardinal Direction ----------------
static const char* headingToCardinal(float deg) {
  static const char* dirs[] = {"N","NE","E","SE","S","SW","W","NW"};
  deg = fmodf(deg, 360.0f);
  if (deg < 0) deg += 360.0f;
  int idx = (int)floor((deg + 22.5f) / 45.0f) % 8;
  return dirs[idx];
}

// Draw/update a small direction label under the status bar (top-right of main area)
static void updateDirectionLabel(const char* dir) {
  static char prev[4] = "---";

  if (strncmp(prev, dir, sizeof(prev)) == 0) return;
  strncpy(prev, dir, sizeof(prev));
  prev[sizeof(prev)-1] = '\0';

  int W = TFT.width();
  int x = W - MARGIN - 70;
  int y = STATUS_H + MARGIN;

  // Clear box
  TFT.fillRect(x, y, 70, 34, WHITE);

  // Right-align the text inside the box
  int len = (int)strlen(dir);
  int charW = 6 * 4;     // approx 6px * textSize(4)
  int textW = len * charW;
  int xText = x + (70 - textW);
  if (xText < x) xText = x;

  TFT.setTextSize(4);
  TFT.setTextColor(BLACK, WHITE);
  TFT.setCursor(xText, y + 2);
  TFT.print(dir);
}

// ---------------- Status Bar ----------------
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

  // Right: BAT
  TFT.setCursor(W - 140, 6);
  TFT.setTextColor(BLACK, GRAY);
  TFT.print("BAT:");
  TFT.print(batteryV, 1);
  TFT.print("V ");
}

// ---------------- Bottom Bar ----------------
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

// ---------------- Arrow Drawing ----------------
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

// ---------------- Demo “Clock” ----------------
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

float demoDist = 41.0f;
float demoVel  = 0.08f;

void setup() {
  Monitor.begin(9600);

  // Bridge (so we can read heading)
  Bridge.begin();
  pythonReady = waitForPython();

  TFT.begin();
  TFT.setRotation(1);
  TFT.fillScreen(WHITE);

  drawStatusBarFrame();
  drawBottomBarFrame();

  updateStatusBar(false, 0, 0.0f);
  updateBottomLeftDistance(demoDist);
  updateBottomRightDateTime(clockSim.yy, clockSim.mm, clockSim.dd,
                            clockSim.hh, clockSim.mi, clockSim.ss);

  // initial direction label
  updateDirectionLabel("--");

  // Optional center dot
  int cx = TFT.width() / 2;
  int cy = (STATUS_H + (TFT.height() - BOTTOM_H)) / 2;
  TFT.fillCircle(cx, cy, 3, BLACK);
}

void loop() {
  Bridge.update();

  static float angle = 0.0f;

  // ---- Demo status values (replace later) ----
  static int sats = 0;
  static bool gpsLock = false;
  static float battV = 8.7f;

  static uint32_t lastStatus = 0;
  if (millis() - lastStatus > 500) {
    lastStatus = millis();
    sats = (sats % 12) + 1;
    gpsLock = (sats > 3);
    updateStatusBar(gpsLock, sats, battV);
  }

  // ---- Demo clock + bottom bar updates ----
  static uint32_t lastBottom = 0;
  if (millis() - lastBottom > 250) {
    lastBottom = millis();

    demoDist += demoVel;
    if (demoDist > 51.0f) demoVel = -fabsf(demoVel);
    if (demoDist < 41.0f) demoVel =  fabsf(demoVel);

    updateBottomLeftDistance(demoDist);

    clockSim.tick();
    updateBottomRightDateTime(clockSim.yy, clockSim.mm, clockSim.dd,
                              clockSim.hh, clockSim.mi, clockSim.ss);
  }

  // ---- NEW: poll heading + update cardinal label ----
  static uint32_t lastHeadingPoll = 0;
  static bool haveHeading = false;

  if (pythonReady && (millis() - lastHeadingPoll > 150)) {
    lastHeadingPoll = millis();

    float heading = -1.0f;
    if (getHeadingDeg(heading)) {
      haveHeading = true;
      angle = heading; // drive arrow from live heading when available
      updateDirectionLabel(headingToCardinal(heading));
    } else {
      haveHeading = false;
      updateDirectionLabel("--");
    }
  }

  // If no live heading is available, keep the old demo rotation
  if (!haveHeading) {
    angle += 3.0f;
    if (angle >= 360.0f) angle -= 360.0f;
  }

  // ---- Arrow draw/erase ----
  if (!isnan(prevAngle)) drawArrow(prevAngle, WHITE);
  drawArrow(angle, MAGENTA);
  prevAngle = angle;

  delay(25);
}
