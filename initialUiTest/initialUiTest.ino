/*
  Pathfinder UI + Touch Buttons (UNO Q + 3.5" ILI9486 Shield)
  - Two on-screen buttons: SET HOME / CLEAR HOME
  - CLEAR HOME -> distance = 0.0m and bottom message: "NO HOME SET"
  - Uses resistive 4-wire touch WITHOUT TouchScreen.h (avoids pins_arduino issues)

  IMPORTANT:
  1) Touch pin mapping depends on your shield. Defaults below are common for many 3.5" UNO shields.
  2) Calibrate TS_MIN/TS_MAX values using the debug prints (tap corners).
*/

#include <DIYables_TFT_Shield.h>
#include <Arduino_RouterBridge.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// ---------------- Colors ----------------
#define WHITE     DIYables_TFT::colorRGB(255, 255, 255)
#define BLACK     DIYables_TFT::colorRGB(0, 0, 0)
#define MAGENTA   DIYables_TFT::colorRGB(255, 0, 255)
#define GREEN     DIYables_TFT::colorRGB(0, 170, 0)
#define RED       DIYables_TFT::colorRGB(200, 0, 0)
#define GRAY      DIYables_TFT::colorRGB(230, 230, 230)
#define BLUE      DIYables_TFT::colorRGB(40, 90, 255)
#define ORANGE    DIYables_TFT::colorRGB(255, 140, 40)

DIYables_TFT_ILI9486_Shield TFT;

// ---------------- Layout ----------------
static const int STATUS_H = 28;
static const int BOTTOM_H = 48;
static const int MARGIN   = 8;

int W, H;
int mainTop, mainBot, mainH;

int arrowCX, arrowCY;
int ARROW_LEN = 48;     // smaller arrow (tweak 40..65)
int ARROW_W   = 28;

int distBoxX, distBoxY, distBoxW, distBoxH;
int distTextX, distTextY;

// Buttons (in main area, above bottom panel)
struct Btn {
  int x, y, w, h;
  const char* label;
  uint16_t fill, border, text;
};
Btn btnSet, btnClear;

// ---------------- Touch configuration ----------------
// These are COMMON for many MCUFRIEND-style 3.5" shields.
// If your touch doesn't respond, you may need to change these.
#define TP_XP 8      // digital
#define TP_YM 9      // digital
#define TP_XM A2     // analog
#define TP_YP A3     // analog

// Calibration: print raw values (Monitor) and adjust.
// Start with these guesses, then tune:
int TS_MINX = 350;
int TS_MAXX = 3800;
int TS_MINY = 350;
int TS_MAXY = 3800;

// If touch is rotated/flipped, change these (try combos):
#define TOUCH_SWAP_XY   1
#define TOUCH_INVERT_X  1
#define TOUCH_INVERT_Y  0

// Debounce
bool touchDown = false;
uint32_t lastTouchMs = 0;

// ---------------- App state ----------------
bool homeSet = false;

// “Navigation outputs” (replace later with real compass+GPS)
float targetAngleDeg = 0.0f;   // 0..360, 0=up
float targetDistM    = 0.0f;

// Smoothed display values
float shownAngleDeg = 0.0f;
float shownDistM    = 0.0f;

float prevArrowAngle = NAN;    // for erase-by-redraw
int   lastDistTenth  = -99999; // update distance at 0.1m steps

// Timing
uint32_t tUI = 0, tBars = 0;
static const uint32_t UI_MS   = 35;   // ~28 FPS (increase if too slow)
static const uint32_t BARS_MS = 400;

// ---------------- Helpers ----------------
static inline float wrap360(float a) {
  while (a < 0) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}
static inline float wrap180(float a) {
  a = wrap360(a);
  if (a > 180.0f) a -= 360.0f;
  return a;
}
static inline float angDiff(float target, float current) {
  return wrap180(target - current); // shortest signed delta [-180..180]
}
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
static long mapLong(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
static bool inRect(int px, int py, const Btn& b) {
  return (px >= b.x && px < b.x + b.w && py >= b.y && py < b.y + b.h);
}

// ---------------- Drawing: Bars ----------------
void drawStatusBar(bool gpsLock, int sats, float battV) {
  TFT.fillRect(0, 0, W, STATUS_H, GRAY);
  TFT.drawLine(0, STATUS_H - 1, W, STATUS_H - 1, BLACK);

  TFT.setTextSize(2);

  TFT.setCursor(MARGIN, 6);
  TFT.setTextColor(gpsLock ? GREEN : RED, GRAY);
  TFT.print("GPS: ");
  TFT.print(gpsLock ? "LOCK" : "--  ");

  TFT.setCursor(W/2 - 40, 6);
  TFT.setTextColor(BLACK, GRAY);
  TFT.print("SAT:");
  TFT.print(sats);
  TFT.print("  ");

  TFT.setCursor(W - 140, 6);
  TFT.print("BAT:");
  TFT.print(battV, 1);
  TFT.print("V ");
}

void drawBottomBarMessage(const char* msg, int errDegSigned) {
  int y0 = H - BOTTOM_H;
  TFT.fillRect(0, y0, W, BOTTOM_H, GRAY);
  TFT.drawLine(0, y0, W, y0, BLACK);

  TFT.setTextSize(3);
  TFT.setCursor(MARGIN, y0 + 12);
  TFT.setTextColor(MAGENTA, GRAY);
  TFT.print(msg);
  TFT.print("     ");

  TFT.setTextSize(2);
  TFT.setCursor(W - 155, y0 + 16);
  TFT.setTextColor(BLACK, GRAY);
  TFT.print("ERR ");
  if (errDegSigned >= 0) TFT.print("+");
  TFT.print(errDegSigned);
  TFT.print((char)247);
  TFT.print("   ");
}

// ---------------- Drawing: Buttons ----------------
void drawButton(const Btn& b) {
  TFT.fillRect(b.x, b.y, b.w, b.h, b.fill);
  TFT.drawRect(b.x, b.y, b.w, b.h, b.border);

  TFT.setTextSize(2);
  TFT.setTextColor(b.text, b.fill);

  // Rough centering
  int labelLen = (int)strlen(b.label);
  int approxCharW = 6 * 2;
  int textW = labelLen * approxCharW;
  int tx = b.x + (b.w - textW) / 2;
  int ty = b.y + (b.h - 16) / 2;

  TFT.setCursor(tx, ty);
  TFT.print(b.label);
}

void flashButton(const Btn& b) {
  // Quick visual feedback
  TFT.drawRect(b.x, b.y, b.w, b.h, BLACK);
  delay(60);
  TFT.drawRect(b.x, b.y, b.w, b.h, b.border);
}

// ---------------- Drawing: Distance ----------------
void drawDistanceFrame() {
  TFT.drawRect(distBoxX, distBoxY, distBoxW, distBoxH, BLACK);
}

void updateDistanceText(float meters) {
  int tenth = (int)(meters * 10.0f + 0.5f);
  if (tenth == lastDistTenth) return;
  lastDistTenth = tenth;

  // Overwrite text with WHITE background (NO big rectangle clear)
  TFT.setTextSize(5);
  TFT.setTextColor(BLACK, WHITE);
  TFT.setCursor(distTextX, distTextY);

  char buf[18];
  // Fixed width helps erase old longer numbers
  snprintf(buf, sizeof(buf), "%6.1f m", meters);
  TFT.print(buf);
}

// ---------------- Drawing: Arrow ----------------
// Triangle arrow, 0°=UP, 90°=RIGHT, 180°=DOWN, 270°=LEFT
void drawArrow(float angleDeg, uint16_t color) {
  float th = angleDeg * (3.1415926f / 180.0f);

  float dx = sinf(th);
  float dy = -cosf(th);

  float px = cosf(th);
  float py = sinf(th);

  int xTip = (int)(arrowCX + dx * ARROW_LEN);
  int yTip = (int)(arrowCY + dy * ARROW_LEN);

  float back = ARROW_LEN * 0.45f;
  int xBaseC = (int)(arrowCX - dx * back);
  int yBaseC = (int)(arrowCY - dy * back);

  int halfW = ARROW_W / 2;
  int xL = (int)(xBaseC - px * halfW);
  int yL = (int)(yBaseC - py * halfW);
  int xR = (int)(xBaseC + px * halfW);
  int yR = (int)(yBaseC + py * halfW);

  TFT.fillTriangle(xTip, yTip, xL, yL, xR, yR, color);
}

void updateArrow(float newAngleDeg) {
  // Erase old arrow by drawing it again in WHITE (smallest possible erase)
  if (!isnan(prevArrowAngle)) {
    drawArrow(prevArrowAngle, WHITE);
  }
  drawArrow(newAngleDeg, MAGENTA);
  prevArrowAngle = newAngleDeg;
}

// ---------------- Touch reading (raw resistive) ----------------
bool readTouchRaw(int &rx, int &ry) {
  // Read X (use YP as ADC)
  pinMode(TP_YP, INPUT);
  pinMode(TP_YM, INPUT);
  pinMode(TP_XP, OUTPUT);
  pinMode(TP_XM, OUTPUT);
  digitalWrite(TP_XP, LOW);
  digitalWrite(TP_XM, HIGH);
  delayMicroseconds(30);
  rx = analogRead(TP_YP);

  // Read Y (use XM as ADC)
  pinMode(TP_XP, INPUT);
  pinMode(TP_XM, INPUT);
  pinMode(TP_YP, OUTPUT);
  pinMode(TP_YM, OUTPUT);
  digitalWrite(TP_YP, HIGH);
  digitalWrite(TP_YM, LOW);
  delayMicroseconds(30);
  ry = analogRead(TP_XM);

  // Release pins
  pinMode(TP_XP, INPUT);
  pinMode(TP_XM, INPUT);
  pinMode(TP_YP, INPUT);
  pinMode(TP_YM, INPUT);

  // Basic “is it touched?” heuristic: ignore extreme values
  if (rx < 50 || ry < 50 || rx > 4090 || ry > 4090) return false;
  return true;
}

bool readTouchScreen(int &sx, int &sy) {
  int rx, ry;
  if (!readTouchRaw(rx, ry)) return false;

  // Map raw to screen coords
  long x = mapLong(rx, TS_MINX, TS_MAXX, 0, W - 1);
  long y = mapLong(ry, TS_MINY, TS_MAXY, 0, H - 1);

  // Apply orientation fixes
#if TOUCH_SWAP_XY
  long t = x; x = y; y = t;
#endif
#if TOUCH_INVERT_X
  x = (W - 1) - x;
#endif
#if TOUCH_INVERT_Y
  y = (H - 1) - y;
#endif

  // Clamp
  if (x < 0) x = 0; if (x >= W) x = W - 1;
  if (y < 0) y = 0; if (y >= H) y = H - 1;

  sx = (int)x;
  sy = (int)y;
  return true;
}

// ---------------- App actions ----------------
void setHomeAction() {
  homeSet = true;
  // In your real code, store current GPS lat/lon as HOME here.

  // UI: keep distance as-is (or set to current)
  drawBottomBarMessage("HOME SET", 0);
}

void clearHomeAction() {
  homeSet = false;

  // distance must become 0 and message "NO HOME SET"
  targetDistM = 0.0f;
  shownDistM  = 0.0f;
  lastDistTenth = -99999;
  updateDistanceText(0.0f);

  // remove arrow (erase last arrow once)
  if (!isnan(prevArrowAngle)) {
    drawArrow(prevArrowAngle, WHITE);
    prevArrowAngle = NAN;
  }

  drawBottomBarMessage("NO HOME SET", 0);
}

// ---------------- Layout init ----------------
void computeLayout() {
  W = TFT.width();
  H = TFT.height();

  mainTop = STATUS_H;
  mainBot = H - BOTTOM_H;
  mainH   = mainBot - mainTop;

  arrowCX = W / 2;
  arrowCY = mainTop + (int)(mainH * 0.40f);

  // Distance box
  distBoxW = (int)(W * 0.70f);
  distBoxH = 60;
  distBoxX = (W - distBoxW) / 2;
  distBoxY = arrowCY + 35;

  distTextX = distBoxX + 18;
  distTextY = distBoxY + 16;

  // Buttons row (near bottom of main area)
  int btnY = mainBot - 44;      // above bottom bar
  int btnW = (W - 3*MARGIN) / 2;
  int btnH = 36;

  btnSet = { MARGIN, btnY, btnW, btnH, "SET HOME",  BLUE,   BLACK, WHITE };
  btnClear = { MARGIN + btnW + MARGIN, btnY, btnW, btnH, "CLEAR", ORANGE, BLACK, BLACK };
}

// Draw static UI once
void drawStaticUI() {
  TFT.fillScreen(WHITE);

  // bars
  drawStatusBar(true, 0, 0.0f);
  drawBottomBarMessage("NO HOME SET", 0);

  // distance box
  drawDistanceFrame();
  updateDistanceText(0.0f);

  // buttons
  drawButton(btnSet);
  drawButton(btnClear);
}

// ---------------- Demo nav (replace later) ----------------
void computeNavDemo() {
  // If home isn’t set: distance stays 0 and no arrow
  if (!homeSet) {
    targetDistM = 0.0f;
    return;
  }

  // Demo: rotate + distance drift
  targetAngleDeg = wrap360(targetAngleDeg + 6.0f); // spinning
  targetDistM += 0.12f;                             // getting farther
  if (targetDistM > 99.9f) targetDistM = 10.0f;
}

// ---------------- Main ----------------
void setup() {
  Monitor.begin(9600);

  TFT.begin();
  TFT.setRotation(1);

  computeLayout();
  drawStaticUI();

  // Start state
  homeSet = false;
  targetAngleDeg = 0.0f;
  targetDistM = 0.0f;
  shownAngleDeg = 0.0f;
  shownDistM = 0.0f;
}

void loop() {
  uint32_t now = millis();

  // --- Touch handling (edge-triggered) ---
  int tx, ty;
  bool pressed = readTouchScreen(tx, ty);

  // Debug (optional): uncomment to calibrate
  // if (pressed) { Monitor.print("touch: "); Monitor.print(tx); Monitor.print(","); Monitor.println(ty); }

  if (pressed && !touchDown && (now - lastTouchMs > 120)) {
    touchDown = true;
    lastTouchMs = now;

    if (inRect(tx, ty, btnSet)) {
      flashButton(btnSet);
      setHomeAction();
    } else if (inRect(tx, ty, btnClear)) {
      flashButton(btnClear);
      clearHomeAction();
    }
  }
  if (!pressed) touchDown = false;

  // --- Nav update (demo for now) ---
  computeNavDemo();

  // --- UI update ---
  if (now - tUI >= UI_MS) {
    tUI = now;

    // Smooth values
    float alphaAng = 0.22f;
    float alphaDist = 0.18f;

    float dA = angDiff(targetAngleDeg, shownAngleDeg);
    shownAngleDeg = wrap360(shownAngleDeg + alphaAng * dA);

    shownDistM += alphaDist * (targetDistM - shownDistM);

    // Arrow: only draw if home set
    if (homeSet) {
      updateArrow(shownAngleDeg);
    }

    // Distance always shown
    updateDistanceText(shownDistM);
  }

  // --- Bars update slower ---
  if (now - tBars >= BARS_MS) {
    tBars = now;

    // Replace with real values later:
    static int sats = 2;
    sats = (sats % 12) + 1;
    bool gpsLock = (sats > 3);
    float batV = 8.7f;

    drawStatusBar(gpsLock, sats, batV);

    if (!homeSet) {
      drawBottomBarMessage("NO HOME SET", 0);
    } else {
      int err = (int)(wrap180(targetAngleDeg) >= 0 ? wrap180(targetAngleDeg) + 0.5f : wrap180(targetAngleDeg) - 0.5f);
      // You can swap this later for your real bearing-heading error
      const char* msg = (err > 15) ? "TURN RIGHT" : (err < -15) ? "TURN LEFT" : "STRAIGHT";
      drawBottomBarMessage(msg, err);
    }

    // Re-draw buttons (keeps them clean if anything overwrote)
    drawButton(btnSet);
    drawButton(btnClear);
  }
}
