/*
  Pathfinder UI (Smooth / Flicker-Free)
  Target: Arduino UNO Q + DIYables_TFT_ILI9486_Shield

  What this does:
  - Smooth arrow direction (continuous rotation, not snap L/R/S)
  - Smooth distance (glides even if GPS updates slowly)
  - No full-screen wipes: only redraws small “dirty” regions (no white overlay)

  Replace the "rawDist/rawErr/gpsLock/sats/batteryV" inputs with your real nav data later.
*/

#include <DIYables_TFT_Shield.h>
#include <Arduino_RouterBridge.h>

#include <math.h>
#include <string.h>
#include <stdio.h>

// ---------- Colors ----------
#define WHITE     DIYables_TFT::colorRGB(255, 255, 255)
#define BLACK     DIYables_TFT::colorRGB(0, 0, 0)
#define MAGENTA   DIYables_TFT::colorRGB(255, 0, 255)
#define GREEN     DIYables_TFT::colorRGB(0, 170, 0)
#define RED       DIYables_TFT::colorRGB(200, 0, 0)
#define GRAY      DIYables_TFT::colorRGB(230, 230, 230)

DIYables_TFT_ILI9486_Shield TFT_display;

// ---------- UI Geometry ----------
static const int STATUS_H = 30;
static const int BOTTOM_H = 50;
static const int MARGIN   = 10;

// Regions around where we draw (tuned for 480x320 landscape)
static const int ARROW_BOX = 220;   // erase box size for arrow region
static const int ARROW_SIZE = 170;  // arrow visual size
static const int DIST_BAND_H = 70;  // erase band height for distance text

// ---------- Timing ----------
static const uint32_t UI_PERIOD_MS     = 50;   // 20 FPS
static const uint32_t NAV_PERIOD_MS    = 100;  // 10 Hz (compass-ish)
static const uint32_t STATUS_PERIOD_MS = 500;  // update status/bottom 2 Hz

// ---------- State for smoothing ----------
static float dispDist = 0.0f;   // displayed distance (m) (smoothed)
static float dispErr  = 0.0f;   // displayed error (deg) (smoothed)

// Dirty tracking (last drawn)
static int   lastDistInt = -9999;
static float lastAngleRad = 9999.0f;
static int   lastTurnBucket = 99;

static bool  lastGpsLock = false;
static int   lastSats = -1;
static int   lastBatTenthV = -1;  // batteryV*10 rounded

// Timers
static uint32_t tUI = 0, tNav = 0, tStatus = 0;

// ---------- Helpers ----------
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static inline float ema(float y, float x, float a) {      // a in [0..1]
  return y + a * (x - y);
}

static inline float slew(float y, float x, float maxStep) {
  float d = x - y;
  if (d >  maxStep) d =  maxStep;
  if (d < -maxStep) d = -maxStep;
  return y + d;
}

// ---------- UI Drawing ----------
void drawStatusBar(int w, bool gpsLock, int sats, float batteryV) {
  TFT_display.fillRect(0, 0, w, STATUS_H, GRAY);
  TFT_display.drawLine(0, STATUS_H - 1, w, STATUS_H - 1, BLACK);

  TFT_display.setTextSize(2);

  // Left: GPS
  TFT_display.setCursor(MARGIN, 6);
  if (gpsLock) {
    TFT_display.setTextColor(GREEN);
    TFT_display.print("GPS: LOCK");
  } else {
    TFT_display.setTextColor(RED);
    TFT_display.print("GPS: --");
  }

  // Middle: SAT
  TFT_display.setTextColor(BLACK);
  TFT_display.setCursor(w / 2 - 55, 6);
  TFT_display.print("SAT: ");
  TFT_display.print(sats);

  // Right: BAT
  TFT_display.setCursor(w - 150, 6);
  TFT_display.print("BAT: ");
  TFT_display.print(batteryV, 1);
  TFT_display.print("V");
}

void drawBottomBar(int w, int h, const char* turnText, int errDeg) {
  int y0 = h - BOTTOM_H;
  TFT_display.fillRect(0, y0, w, BOTTOM_H, GRAY);
  TFT_display.drawLine(0, y0, w, y0, BLACK);

  // Left: TURN text
  TFT_display.setTextSize(3);
  TFT_display.setTextColor(MAGENTA);
  TFT_display.setCursor(MARGIN, y0 + 10);
  TFT_display.print(turnText);

  // Right: error
  TFT_display.setTextSize(2);
  TFT_display.setTextColor(BLACK);
  TFT_display.setCursor(w - 170, y0 + 18);
  TFT_display.print("ERR ");
  if (errDeg >= 0) TFT_display.print("+");
  TFT_display.print(errDeg);
  TFT_display.print((char)247);
}

// A fast, clean rotated arrow (a navigation pointer). Triangle + small tail triangle.
void drawArrowRot(int cx, int cy, int size, float angRad, uint16_t fill, uint16_t outline) {
  // Triangle pointer in local coords, pointing UP (negative y)
  float L = size * 0.58f;    // length
  float W = size * 0.40f;    // width

  // tip, left, right
  float x0 = 0.0f,    y0 = -L;
  float x1 = -W/2.0f, y1 =  L*0.35f;
  float x2 =  W/2.0f, y2 =  L*0.35f;

  // Small tail to make it look more arrow-like
  float tW = W * 0.30f;
  float tL = L * 0.35f;
  float x3 = -tW/2.0f, y3 =  L*0.35f;        // tail left top (connect point)
  float x4 =  tW/2.0f, y4 =  L*0.35f;        // tail right top
  float x5 =  0.0f,    y5 =  L*0.35f + tL;   // tail bottom point

  auto RX = [&](float x, float y){ return x*cosf(angRad) - y*sinf(angRad); };
  auto RY = [&](float x, float y){ return x*sinf(angRad) + y*cosf(angRad); };

  int X0 = cx + (int)RX(x0,y0), Y0 = cy + (int)RY(x0,y0);
  int X1 = cx + (int)RX(x1,y1), Y1 = cy + (int)RY(x1,y1);
  int X2 = cx + (int)RX(x2,y2), Y2 = cy + (int)RY(x2,y2);

  int X3 = cx + (int)RX(x3,y3), Y3 = cy + (int)RY(x3,y3);
  int X4 = cx + (int)RX(x4,y4), Y4 = cy + (int)RY(x4,y4);
  int X5 = cx + (int)RX(x5,y5), Y5 = cy + (int)RY(x5,y5);

  // Fill
  TFT_display.fillTriangle(X0,Y0, X1,Y1, X2,Y2, fill);
  TFT_display.fillTriangle(X3,Y3, X4,Y4, X5,Y5, fill);

  // Outline (crisper)
  TFT_display.drawTriangle(X0,Y0, X1,Y1, X2,Y2, outline);
  TFT_display.drawTriangle(X3,Y3, X4,Y4, X5,Y5, outline);
}

void updateArrowRegion(int w, int h, float errDeg) {
  int yTop = STATUS_H;
  int yBot = h - BOTTOM_H;
  int areaH = yBot - yTop;

  int cx = w / 2;
  int cy = yTop + areaH / 2 - 40;

  // Clamp how far the arrow rotates (keeps it readable)
  float angDeg = clampf(errDeg, -75.0f, 75.0f);
  float angRad = angDeg * 3.1415926f / 180.0f;

  // Only redraw if angle changed enough (~1.5 degrees)
  if (fabsf(angRad - lastAngleRad) > 0.026f) {
    TFT_display.fillRect(cx - ARROW_BOX/2, cy - ARROW_BOX/2, ARROW_BOX, ARROW_BOX, WHITE);
    drawArrowRot(cx, cy, ARROW_SIZE, angRad, MAGENTA, BLACK);
    lastAngleRad = angRad;
  }
}

void updateDistanceRegion(int w, int h, float distMeters) {
  int yTop = STATUS_H;
  int yBot = h - BOTTOM_H;
  int areaH = yBot - yTop;

  int cx = w / 2;
  int cy = yTop + areaH / 2 - 40;
  int ty = cy + 110;

  int distInt = (int)(distMeters + 0.5f);
  if (distInt != lastDistInt) {
    // Clear band where distance is printed
    TFT_display.fillRect(0, ty - 10, w, DIST_BAND_H, WHITE);

    TFT_display.setTextColor(BLACK);
    TFT_display.setTextSize(6);

    char buf[16];
    snprintf(buf, sizeof(buf), "%d m", distInt);

    // Rough centering
    int len = (int)strlen(buf);
    int approxCharW = 6 * 6;
    int textW = len * approxCharW;
    int tx = (w - textW) / 2;

    TFT_display.setCursor(tx, ty);
    TFT_display.print(buf);

    lastDistInt = distInt;
  }
}

int turnBucketFromErr(float errDeg) {
  if (errDeg > 15) return 1;     // Right
  if (errDeg < -15) return -1;   // Left
  return 0;                      // Straight
}

const char* turnTextFromBucket(int b) {
  if (b > 0) return "TURN RIGHT";
  if (b < 0) return "TURN LEFT";
  return "STRAIGHT";
}

// Call this at startup to paint the static background once
void drawInitialBackground(int w, int h) {
  TFT_display.fillScreen(WHITE);
  // Draw status and bottom bars once (will also redraw later on changes)
  drawStatusBar(w, true, 0, 0.0f);
  drawBottomBar(w, h, "STRAIGHT", 0);
}

void setup() {
  Monitor.begin(9600);
  Monitor.println("Pathfinder UI (Smooth) - UNO Q");

  TFT_display.begin();
  TFT_display.setRotation(1);  // landscape

  int w = TFT_display.width();
  int h = TFT_display.height();
  drawInitialBackground(w, h);

  // Initialize displayed values to something sensible
  dispDist = 42.0f;
  dispErr  = 0.0f;
}

void loop() {
  uint32_t now = millis();

  // -----------------------------
  // 1) NAV UPDATE (replace inputs)
  // -----------------------------
  if (now - tNav >= NAV_PERIOD_MS) {
    tNav = now;

    // ===== Replace these with your real values =====
    // rawDist: distance to HOME in meters (from GPS)
    // rawErr:  bearing - heading in degrees (-180..+180)
    // gpsLock/sats/batteryV: your status values
    static float rawDist = 42.0f;
    static float rawErr  = 35.0f;

    // Simple demo motion (so you can see smooth changes)
    rawErr += 7.0f; if (rawErr > 80.0f) rawErr = -80.0f;
    rawDist -= 0.35f; if (rawDist < 5.0f) rawDist = 42.0f;

    // Smooth distance & error (tune to taste)
    dispDist = ema(dispDist, rawDist, 0.18f);
    dispDist = slew(dispDist, rawDist, 1.2f);   // max 1.2m per NAV tick

    dispErr  = ema(dispErr, rawErr, 0.25f);
    dispErr  = slew(dispErr, rawErr, 7.0f);     // max 7° per NAV tick
  }

  // -----------------------------
  // 2) STATUS/BOTTOM UPDATE (slower)
  // -----------------------------
  if (now - tStatus >= STATUS_PERIOD_MS) {
    tStatus = now;

    // ===== Replace these with your real values =====
    static bool gpsLock = true;
    static int sats = 10;
    static float batteryV = 8.7f;

    // Demo changes:
    sats = (sats % 12) + 1;
    gpsLock = (sats > 3);

    // Decide turn text bucket from *smoothed* error
    int bucket = turnBucketFromErr(dispErr);
    const char* turnText = turnTextFromBucket(bucket);
    int errInt = (int)(dispErr + (dispErr >= 0 ? 0.5f : -0.5f));

    int w = TFT_display.width();
    int h = TFT_display.height();

    // Redraw status bar only if changed
    int batTenth = (int)(batteryV * 10.0f + 0.5f);
    if (gpsLock != lastGpsLock || sats != lastSats || batTenth != lastBatTenthV) {
      drawStatusBar(w, gpsLock, sats, batteryV);
      lastGpsLock = gpsLock;
      lastSats = sats;
      lastBatTenthV = batTenth;
    }

    // Redraw bottom bar only if turn bucket changed OR error int changed a lot
    if (bucket != lastTurnBucket) {
      drawBottomBar(w, h, turnText, errInt);
      lastTurnBucket = bucket;
    } else {
      // still refresh bottom bar occasionally for ERR number if you want
      // (optional) uncomment if you want ERR to update even when bucket same:
      // drawBottomBar(w, h, turnText, errInt);
    }
  }

  // -----------------------------
  // 3) UI REFRESH (smooth + dirty)
  // -----------------------------
  if (now - tUI >= UI_PERIOD_MS) {
    tUI = now;
    int w = TFT_display.width();
    int h = TFT_display.height();

    updateArrowRegion(w, h, dispErr);
    updateDistanceRegion(w, h, dispDist);
  }
}
