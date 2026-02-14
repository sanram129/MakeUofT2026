/*
  Pathfinder UI + Touch Buttons (UNO Q + 3.5" ILI9486 Shield)
  - SET HOME / CLEAR HOME
  - CLEAR HOME: distance -> 0.0m and bottom says "NO HOME SET"
  - Raw resistive touch read (no TouchScreen.h)

  NOTE: ADC on UNO Q is 0..4095 (12-bit).
*/

#include <DIYables_TFT_Shield.h>
#include <Arduino_RouterBridge.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// ---------- TFT bus mapping (common MCUFRIEND 3.5" shields) ----------
#define LCD_RD  A0
#define LCD_WR  A1
#define LCD_RS  A2
#define LCD_CS  A3
#define LCD_RST A4

#define LCD_D0  8
#define LCD_D1  9
#define LCD_D2  2
#define LCD_D3  3
#define LCD_D4  4
#define LCD_D5  5
#define LCD_D6  6
#define LCD_D7  7

// ---------- Touch pins (shared with TFT pins on these shields) ----------
#define TP_XP  8    // D0
#define TP_YM  9    // D1
#define TP_XM  A2   // RS
#define TP_YP  A3   // CS

// ---------- Safe TFT bus takeover/restore during touch reads ----------
void tftBusTakeoverForTouch() {
  pinMode(LCD_CS, OUTPUT); digitalWrite(LCD_CS, HIGH); // deselect TFT
  pinMode(LCD_RD, OUTPUT); digitalWrite(LCD_RD, HIGH);
  pinMode(LCD_WR, OUTPUT); digitalWrite(LCD_WR, HIGH);
}

void tftBusRestoreAfterTouch() {
  // Put bus back so TFT library can work normally again.
  pinMode(LCD_CS, OUTPUT);  digitalWrite(LCD_CS, LOW);   // many shields keep CS low
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_RD, OUTPUT);  digitalWrite(LCD_RD, HIGH);
  pinMode(LCD_WR, OUTPUT);  digitalWrite(LCD_WR, HIGH);
  pinMode(LCD_RST, OUTPUT); digitalWrite(LCD_RST, HIGH);

  pinMode(LCD_D0, OUTPUT);
  pinMode(LCD_D1, OUTPUT);
  pinMode(LCD_D2, OUTPUT);
  pinMode(LCD_D3, OUTPUT);
  pinMode(LCD_D4, OUTPUT);
  pinMode(LCD_D5, OUTPUT);
  pinMode(LCD_D6, OUTPUT);
  pinMode(LCD_D7, OUTPUT);
}

// ---------- Colors ----------
#define WHITE     DIYables_TFT::colorRGB(255, 255, 255)
#define BLACK     DIYables_TFT::colorRGB(0, 0, 0)
#define MAGENTA   DIYables_TFT::colorRGB(255, 0, 255)
#define GREEN     DIYables_TFT::colorRGB(0, 170, 0)
#define RED       DIYables_TFT::colorRGB(200, 0, 0)
#define GRAY      DIYables_TFT::colorRGB(230, 230, 230)
#define BLUE      DIYables_TFT::colorRGB(40, 90, 255)
#define ORANGE    DIYables_TFT::colorRGB(255, 140, 40)

DIYables_TFT_ILI9486_Shield TFT;

// ---------- Layout ----------
static const int STATUS_H = 28;
static const int BOTTOM_H = 48;
static const int MARGIN   = 8;

int W, H;
int mainTop, mainBot, mainH;

int arrowCX, arrowCY;
int ARROW_LEN = 48;
int ARROW_W   = 28;

int distBoxX, distBoxY, distBoxW, distBoxH;
int distTextX, distTextY;

struct Btn {
  int x, y, w, h;
  const char* label;
  uint16_t fill, border, text;
};
Btn btnSet, btnClear;

// ---------- Touch calibration (12-bit ADC: 0..4095) ----------
int TS_MINX = 350;
int TS_MAXX = 3800;
int TS_MINY = 350;
int TS_MAXY = 3800;

#define TOUCH_SWAP_XY   1
#define TOUCH_INVERT_X  1
#define TOUCH_INVERT_Y  0

bool touchDown = false;
uint32_t lastTouchMs = 0;

// ---------- App state ----------
bool homeSet = false;
float targetAngleDeg = 0.0f;
float targetDistM    = 0.0f;

float shownAngleDeg = 0.0f;
float shownDistM    = 0.0f;

float prevArrowAngle = NAN;
int   lastDistTenth  = -99999;

uint32_t tUI = 0, tBars = 0;
static const uint32_t UI_MS   = 35;
static const uint32_t BARS_MS = 400;

// ---------- Helpers ----------
static inline float wrap360(float a) { while (a < 0) a += 360; while (a >= 360) a -= 360; return a; }
static inline float wrap180(float a) { a = wrap360(a); if (a > 180) a -= 360; return a; }
static inline float angDiff(float target, float current) { return wrap180(target - current); }

static long mapLong(long x, long in_min, long in_max, long out_min, long out_max) {
  if (in_max == in_min) return out_min;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
static bool inRect(int px, int py, const Btn& b) {
  return (px >= b.x && px < b.x + b.w && py >= b.y && py < b.y + b.h);
}

// ---------- Draw UI ----------
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

void drawButton(const Btn& b) {
  TFT.fillRect(b.x, b.y, b.w, b.h, b.fill);
  TFT.drawRect(b.x, b.y, b.w, b.h, b.border);

  TFT.setTextSize(2);
  TFT.setTextColor(b.text, b.fill);

  int labelLen = (int)strlen(b.label);
  int approxCharW = 12; // ~6px * size2
  int textW = labelLen * approxCharW;
  int tx = b.x + (b.w - textW) / 2;
  int ty = b.y + (b.h - 16) / 2;

  TFT.setCursor(tx, ty);
  TFT.print(b.label);
}

void flashButton(const Btn& b) {
  TFT.drawRect(b.x, b.y, b.w, b.h, BLACK);
  delay(60);
  TFT.drawRect(b.x, b.y, b.w, b.h, b.border);
}

void drawDistanceFrame() {
  TFT.drawRect(distBoxX, distBoxY, distBoxW, distBoxH, BLACK);
}

void updateDistanceText(float meters) {
  int tenth = (int)(meters * 10.0f + 0.5f);
  if (tenth == lastDistTenth) return;
  lastDistTenth = tenth;

  TFT.setTextSize(5);
  TFT.setTextColor(BLACK, WHITE);
  TFT.setCursor(distTextX, distTextY);

  char buf[18];
  snprintf(buf, sizeof(buf), "%6.1f m", meters);
  TFT.print(buf);
}

void drawArrow(float angleDeg, uint16_t color) {
  float th = angleDeg * (3.1415926f / 180.0f);

  float dx = sinf(th), dy = -cosf(th);
  float px = cosf(th), py = sinf(th);

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
  if (!isnan(prevArrowAngle)) drawArrow(prevArrowAngle, WHITE);
  drawArrow(newAngleDeg, MAGENTA);
  prevArrowAngle = newAngleDeg;
}

// ---------- Touch ----------
bool readTouchRaw(int &rx, int &ry) {
  static uint32_t lastPoll = 0;
  if (millis() - lastPoll < 80) return false; // slow polling is safer + enough for buttons
  lastPoll = millis();

  tftBusTakeoverForTouch();

  // X
  pinMode(TP_YP, INPUT);
  pinMode(TP_YM, INPUT);
  pinMode(TP_XP, OUTPUT);
  pinMode(TP_XM, OUTPUT);
  digitalWrite(TP_XP, LOW);
  digitalWrite(TP_XM, HIGH);
  delayMicroseconds(30);
  rx = analogRead(TP_YP);

  // Y
  pinMode(TP_XP, INPUT);
  pinMode(TP_XM, INPUT);
  pinMode(TP_YP, OUTPUT);
  pinMode(TP_YM, OUTPUT);
  digitalWrite(TP_YP, HIGH);
  digitalWrite(TP_YM, LOW);
  delayMicroseconds(30);
  ry = analogRead(TP_XM);

  tftBusRestoreAfterTouch();

  if (rx < 80 || ry < 80) return false;
  if (rx > 4010 || ry > 4010) return false;
  return true;
}

bool readTouchScreen(int &sx, int &sy) {
  int rx, ry;
  if (!readTouchRaw(rx, ry)) return false;

  long x = mapLong(rx, TS_MINX, TS_MAXX, 0, W - 1);
  long y = mapLong(ry, TS_MINY, TS_MAXY, 0, H - 1);

#if TOUCH_SWAP_XY
  long t = x; x = y; y = t;
#endif
#if TOUCH_INVERT_X
  x = (W - 1) - x;
#endif
#if TOUCH_INVERT_Y
  y = (H - 1) - y;
#endif

  if (x < 0) x = 0; if (x >= W) x = W - 1;
  if (y < 0) y = 0; if (y >= H) y = H - 1;

  sx = (int)x;
  sy = (int)y;
  return true;
}

// ---------- Actions ----------
void setHomeAction() {
  homeSet = true;
  drawBottomBarMessage("HOME SET", 0);
}

void clearHomeAction() {
  homeSet = false;

  targetDistM = 0.0f;
  shownDistM  = 0.0f;
  lastDistTenth = -99999;
  updateDistanceText(0.0f);

  if (!isnan(prevArrowAngle)) {
    drawArrow(prevArrowAngle, WHITE);
    prevArrowAngle = NAN;
  }

  drawBottomBarMessage("NO HOME SET", 0);
}

// ---------- Layout ----------
void computeLayout() {
  W = TFT.width();
  H = TFT.height();

  mainTop = STATUS_H;
  mainBot = H - BOTTOM_H;
  mainH   = mainBot - mainTop;

  arrowCX = W / 2;
  arrowCY = mainTop + (int)(mainH * 0.40f);

  distBoxW = (int)(W * 0.70f);
  distBoxH = 60;
  distBoxX = (W - distBoxW) / 2;
  distBoxY = arrowCY + 35;

  distTextX = distBoxX + 18;
  distTextY = distBoxY + 16;

  int btnY = mainBot - 44;
  int btnW = (W - 3 * MARGIN) / 2;
  int btnH = 36;

  btnSet   = { MARGIN, btnY, btnW, btnH, "SET HOME", BLUE,   BLACK, WHITE };
  btnClear = { MARGIN + btnW + MARGIN, btnY, btnW, btnH, "CLEAR",    ORANGE, BLACK, BLACK };
}

void drawStaticUI() {
  TFT.fillScreen(WHITE);
  drawStatusBar(true, 0, 0.0f);
  drawBottomBarMessage("NO HOME SET", 0);
  drawDistanceFrame();
  updateDistanceText(0.0f);
  drawButton(btnSet);
  drawButton(btnClear);
}

// Demo navigation (replace later)
void computeNavDemo() {
  if (!homeSet) { targetDistM = 0.0f; return; }
  targetAngleDeg = wrap360(targetAngleDeg + 6.0f);
  targetDistM += 0.12f;
  if (targetDistM > 99.9f) targetDistM = 10.0f;
}

// ---------- Arduino ----------
void setup() {
  Monitor.begin(9600);

  TFT.begin();
  TFT.setRotation(1);

  computeLayout();
  drawStaticUI();

  homeSet = false;
  targetAngleDeg = 0.0f;
  targetDistM = 0.0f;
  shownAngleDeg = 0.0f;
  shownDistM = 0.0f;
}

void loop() {
  uint32_t now = millis();

  // Touch (edge-triggered)
  int tx, ty;
  bool pressed = readTouchScreen(tx, ty);

  if (pressed && !touchDown && (now - lastTouchMs > 150)) {
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

  // Demo nav
  computeNavDemo();

  // UI update
  if (now - tUI >= UI_MS) {
    tUI = now;

    float alphaAng  = 0.22f;
    float alphaDist = 0.18f;

    float dA = angDiff(targetAngleDeg, shownAngleDeg);
    shownAngleDeg = wrap360(shownAngleDeg + alphaAng * dA);
    shownDistM += alphaDist * (targetDistM - shownDistM);

    if (homeSet) updateArrow(shownAngleDeg);
    updateDistanceText(shownDistM);
  }

  // Bars slower
  if (now - tBars >= BARS_MS) {
    tBars = now;

    static int sats = 2;
    sats = (sats % 12) + 1;
    bool gpsLock = (sats > 3);
    float batV = 8.7f;

    drawStatusBar(gpsLock, sats, batV);

    if (!homeSet) {
      drawBottomBarMessage("NO HOME SET", 0);
    } else {
      int err = (int)(wrap180(targetAngleDeg) >= 0 ? wrap180(targetAngleDeg) + 0.5f : wrap180(targetAngleDeg) - 0.5f);
      const char* msg = (err > 15) ? "TURN RIGHT" : (err < -15) ? "TURN LEFT" : "STRAIGHT";
      drawBottomBarMessage(msg, err);
    }

    drawButton(btnSet);
    drawButton(btnClear);
  }
}
