/*
  Pathfinder UI: Smooth 0–360° Arrow + Smooth Distance (No Big White Wipe)
  Target: Arduino UNO Q + DIYables_TFT_ILI9486_Shield

  How to use with your real nav:
    rawErrDeg = wrap180(bearingToHomeDeg - headingDeg);   // -180..+180 (right positive)
    rawDistM  = distanceToHomeMeters;
*/

#include <DIYables_TFT_Shield.h>
#include <Arduino_RouterBridge.h>

#include <math.h>
#include <string.h>
#include <stdio.h>

// Colors
#define WHITE     DIYables_TFT::colorRGB(255, 255, 255)
#define BLACK     DIYables_TFT::colorRGB(0, 0, 0)
#define MAGENTA   DIYables_TFT::colorRGB(255, 0, 255)
#define GREEN     DIYables_TFT::colorRGB(0, 170, 0)
#define RED       DIYables_TFT::colorRGB(200, 0, 0)
#define GRAY      DIYables_TFT::colorRGB(230, 230, 230)

DIYables_TFT_ILI9486_Shield TFT_display;

// UI Geometry
static const int STATUS_H = 30;
static const int BOTTOM_H = 50;
static const int MARGIN   = 10;

// Layout (computed after begin)
static int W, H;
static int mainTop, mainBot, mainH;
static int arrowCX, arrowCY;
static int arrowSize;

// Distance “card” region (fixed box so redraw is clean)
static int distBoxX, distBoxY, distBoxW, distBoxH;

// Timing
static const uint32_t ARROW_PERIOD_MS  = 40;   // 25 FPS arrow
static const uint32_t DIST_PERIOD_MS   = 120;  // distance update ~8 Hz
static const uint32_t STATUS_PERIOD_MS = 500;  // status/bottom ~2 Hz

static uint32_t tArrow=0, tDist=0, tStatus=0;

// Smoothed display values
static float dispAngleDeg = 0.0f;  // 0..360 (0 = up, 90 = right, 180 = down)
static float dispDistM    = 0.0f;

// Last drawn values (for minimal redraw)
static float lastDrawnAngleDeg = 9999.0f;
static int   lastDrawnDistInt  = -9999;

// ---------------- Angle helpers ----------------
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

// shortest signed difference from current -> target (degrees), in [-180, +180]
static inline float angDiffDeg(float target, float current) {
  return wrap180(target - current);
}

// exponential smoothing on angles, correctly handling wrap-around
static float smoothAngleDeg(float current, float target, float alpha) {
  float d = angDiffDeg(target, current);      // shortest way
  return wrap360(current + alpha * d);
}

// ---------------- UI Draw ----------------
void drawStatusBar(bool gpsLock, int sats, float batteryV) {
  TFT_display.fillRect(0, 0, W, STATUS_H, GRAY);
  TFT_display.drawLine(0, STATUS_H - 1, W, STATUS_H - 1, BLACK);

  TFT_display.setTextSize(2);

  // GPS
  TFT_display.setCursor(MARGIN, 6);
  if (gpsLock) { TFT_display.setTextColor(GREEN); TFT_display.print("GPS: LOCK"); }
  else         { TFT_display.setTextColor(RED);   TFT_display.print("GPS: --");   }

  // SAT
  TFT_display.setTextColor(BLACK);
  TFT_display.setCursor(W/2 - 55, 6);
  TFT_display.print("SAT: ");
  TFT_display.print(sats);

  // BAT
  TFT_display.setCursor(W - 150, 6);
  TFT_display.print("BAT: ");
  TFT_display.print(batteryV, 1);
  TFT_display.print("V");
}

void drawBottomBar(const char* turnText, int errDegSigned) {
  int y0 = H - BOTTOM_H;
  TFT_display.fillRect(0, y0, W, BOTTOM_H, GRAY);
  TFT_display.drawLine(0, y0, W, y0, BLACK);

  TFT_display.setTextSize(3);
  TFT_display.setTextColor(MAGENTA);
  TFT_display.setCursor(MARGIN, y0 + 10);
  TFT_display.print(turnText);

  TFT_display.setTextSize(2);
  TFT_display.setTextColor(BLACK);
  TFT_display.setCursor(W - 170, y0 + 18);
  TFT_display.print("ERR ");
  if (errDegSigned >= 0) TFT_display.print("+");
  TFT_display.print(errDegSigned);
  TFT_display.print((char)247);
}

// Rotating arrow (0°=up, 90°=right). Drawn as triangles (fast).
// To erase without flicker, draw the SAME arrow again using WHITE fill+WHITE outline.
void drawArrowRot(float angleDeg, uint16_t fill, uint16_t outline) {
  float rad = angleDeg * 3.1415926f / 180.0f;

  // Arrow shape in local coords (pointing up at angleDeg=0)
  float L = arrowSize * 0.60f;   // length
  float Wd = arrowSize * 0.42f;  // width
  float tailW = Wd * 0.30f;
  float tailL = L * 0.40f;

  // Main pointer triangle: tip, left, right
  float x0=0,      y0=-L;
  float x1=-Wd/2,  y1= L*0.35f;
  float x2= Wd/2,  y2= L*0.35f;

  // Tail triangle: makes it look like a nav arrow
  float x3=-tailW/2, y3=L*0.35f;
  float x4= tailW/2, y4=L*0.35f;
  float x5= 0,       y5=L*0.35f + tailL;

  auto RX = [&](float x, float y){ return x*cosf(rad) - y*sinf(rad); };
  auto RY = [&](float x, float y){ return x*sinf(rad) + y*cosf(rad); };

  int X0 = arrowCX + (int)RX(x0,y0), Y0 = arrowCY + (int)RY(x0,y0);
  int X1 = arrowCX + (int)RX(x1,y1), Y1 = arrowCY + (int)RY(x1,y1);
  int X2 = arrowCX + (int)RX(x2,y2), Y2 = arrowCY + (int)RY(x2,y2);

  int X3 = arrowCX + (int)RX(x3,y3), Y3 = arrowCY + (int)RY(x3,y3);
  int X4 = arrowCX + (int)RX(x4,y4), Y4 = arrowCY + (int)RY(x4,y4);
  int X5 = arrowCX + (int)RX(x5,y5), Y5 = arrowCY + (int)RY(x5,y5);

  TFT_display.fillTriangle(X0,Y0, X1,Y1, X2,Y2, fill);
  TFT_display.fillTriangle(X3,Y3, X4,Y4, X5,Y5, fill);

  TFT_display.drawTriangle(X0,Y0, X1,Y1, X2,Y2, outline);
  TFT_display.drawTriangle(X3,Y3, X4,Y4, X5,Y5, outline);
}

void drawDistanceCard(int distInt) {
  // Draw a “card” (fixed rectangle) so clearing looks intentional and small
  TFT_display.fillRect(distBoxX, distBoxY, distBoxW, distBoxH, WHITE);
  TFT_display.drawRect(distBoxX, distBoxY, distBoxW, distBoxH, BLACK);

  TFT_display.setTextColor(BLACK);
  TFT_display.setTextSize(6);

  char buf[16];
  snprintf(buf, sizeof(buf), "%d m", distInt);

  // crude centering
  int len = (int)strlen(buf);
  int approxCharW = 6 * 6;
  int textW = len * approxCharW;
  int tx = distBoxX + (distBoxW - textW) / 2;
  int ty = distBoxY + 8;

  TFT_display.setCursor(tx, ty);
  TFT_display.print(buf);
}

// ---------------- Setup layout ----------------
void computeLayout() {
  W = TFT_display.width();
  H = TFT_display.height();

  mainTop = STATUS_H;
  mainBot = H - BOTTOM_H;
  mainH   = mainBot - mainTop;

  arrowCX = W / 2;
  arrowCY = mainTop + (int)(mainH * 0.42f);   // a bit above center
  arrowSize = (H < 360) ? 170 : 200;          // decent default; tweak if you want

  // Distance box near bottom of main area (doesn't overlap arrow)
  distBoxW = 260;
  distBoxH = 70;
  distBoxX = (W - distBoxW) / 2;
  distBoxY = mainTop + (int)(mainH * 0.68f);
}

// ---------------- Main loop updates ----------------
void updateArrowSmooth(float targetAngleDeg) {
  // Smooth target -> displayed
  dispAngleDeg = smoothAngleDeg(dispAngleDeg, targetAngleDeg, 0.25f);

  // Only redraw if changed enough (~1 deg)
  float d = fabsf(angDiffDeg(dispAngleDeg, lastDrawnAngleDeg));
  if (d > 1.0f) {
    // Erase old arrow by redrawing it in WHITE (no white rectangle flicker)
    if (lastDrawnAngleDeg != 9999.0f) {
      drawArrowRot(lastDrawnAngleDeg, WHITE, WHITE);
    }
    // Draw new arrow
    drawArrowRot(dispAngleDeg, MAGENTA, BLACK);

    lastDrawnAngleDeg = dispAngleDeg;
  }
}

void updateDistanceSmooth(float targetDistM) {
  // Smooth distance (EMA + a little slew limiting)
  float alpha = 0.18f;
  float next = dispDistM + alpha * (targetDistM - dispDistM);

  // Slew limit so it glides (meters per update)
  float maxStep = 1.0f;
  float step = next - dispDistM;
  if (step >  maxStep) next = dispDistM + maxStep;
  if (step < -maxStep) next = dispDistM - maxStep;

  dispDistM = next;

  int distInt = (int)(dispDistM + 0.5f);
  if (distInt != lastDrawnDistInt) {
    drawDistanceCard(distInt);
    lastDrawnDistInt = distInt;
  }
}

const char* turnTextFromErr(float errDegSigned) {
  if (errDegSigned > 15) return "TURN RIGHT";
  if (errDegSigned < -15) return "TURN LEFT";
  return "STRAIGHT";
}

void setup() {
  Monitor.begin(9600);
  Monitor.println("Pathfinder UI: Smooth 360 Arrow");

  TFT_display.begin();
  TFT_display.setRotation(1);   // landscape
  TFT_display.fillScreen(WHITE);

  computeLayout();

  // Draw static UI once
  drawStatusBar(true, 10, 8.7f);
  drawBottomBar("STRAIGHT", 0);

  // Initialize smooth values
  dispAngleDeg = 0.0f;
  dispDistM = 42.0f;

  // First draw
  drawArrowRot(dispAngleDeg, MAGENTA, BLACK);
  lastDrawnAngleDeg = dispAngleDeg;

  drawDistanceCard((int)(dispDistM + 0.5f));
  lastDrawnDistInt = (int)(dispDistM + 0.5f);
}

void loop() {
  uint32_t now = millis();

  // --------------------
  // DEMO INPUTS (replace)
  // --------------------
  // rawErrDeg: -180..+180 (right positive)
  // rawDistM : meters
  static float rawErrDeg = 0.0f;
  static float rawDistM  = 42.0f;

  // Demo: rotate full 360 and distance decreasing
  rawErrDeg += 7.0f;
  if (rawErrDeg > 180.0f) rawErrDeg = -180.0f;

  rawDistM -= 0.25f;
  if (rawDistM < 5.0f) rawDistM = 42.0f;

  // Convert signed error (-180..180) to arrow angle (0..360) where:
  // 0 = up/forward, 90 = right, 180 = down/back, 270 = left
  float targetAngleDeg = wrap360(rawErrDeg);
  float errSigned = wrap180(rawErrDeg);

  // --------------------
  // Smooth updates
  // --------------------
  if (now - tArrow >= ARROW_PERIOD_MS) {
    tArrow = now;
    updateArrowSmooth(targetAngleDeg);
  }

  if (now - tDist >= DIST_PERIOD_MS) {
    tDist = now;
    updateDistanceSmooth(rawDistM);
  }

  if (now - tStatus >= STATUS_PERIOD_MS) {
    tStatus = now;
    // Replace these with real values later
    static bool gpsLock = true;
    static int sats = 10;
    static float batV = 8.7f;

    // demo variation
    sats = (sats % 12) + 1;
    gpsLock = (sats > 3);

    drawStatusBar(gpsLock, sats, batV);
    drawBottomBar(turnTextFromErr(errSigned), (int)(errSigned >= 0 ? errSigned + 0.5f : errSigned - 0.5f));
  }
}
