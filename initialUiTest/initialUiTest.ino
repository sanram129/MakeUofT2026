/*
  Pathfinder UI: Smooth 0–360 Arrow + Smooth Distance (FAST)
  Arduino UNO Q + DIYables_TFT_ILI9486_Shield

  Replace demo inputs with real nav later:
    targetAngleDeg = wrap360(bearingToHomeDeg - headingDeg);  // 0..360
    targetDistM    = distanceToHomeMeters;
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

// UI
static const int STATUS_H = 30;
static const int BOTTOM_H = 50;
static const int MARGIN   = 10;

// Screen/layout
static int W, H;
static int mainTop, mainBot, mainH;
static int arrowCX, arrowCY;
static int ARROW_SIZE = 110;      // smaller (tweak 90..140)
static int distBoxX, distBoxY, distBoxW, distBoxH;

// Timing (fast + smooth)
static const uint32_t UI_MS      = 16;   // ~60 FPS
static const uint32_t STATUS_MS  = 500;  // slow status redraw
static uint32_t tUI = 0, tStatus = 0;

// Displayed (smoothed) values
static float dispAngle = 0.0f;   // 0..360
static float dispDist  = 41.0f;  // meters

// Last drawn
static float lastDrawAngle = 9999.0f;
static int   lastDistTenth = -9999;

// ---------- Helpers ----------
static inline float wrap360(float a){
  while (a < 0) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}
static inline float wrap180(float a){
  a = wrap360(a);
  if (a > 180.0f) a -= 360.0f;
  return a;
}
static inline float angDiff(float target, float current){
  return wrap180(target - current);  // shortest signed diff
}
static inline float clampf(float x, float lo, float hi){
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// ---------- Bars ----------
void drawStatusBar(bool gpsLock, int sats, float batteryV) {
  TFT_display.fillRect(0, 0, W, STATUS_H, GRAY);
  TFT_display.drawLine(0, STATUS_H - 1, W, STATUS_H - 1, BLACK);

  TFT_display.setTextSize(2);

  TFT_display.setCursor(MARGIN, 6);
  if (gpsLock) { TFT_display.setTextColor(GREEN); TFT_display.print("GPS: LOCK"); }
  else         { TFT_display.setTextColor(RED);   TFT_display.print("GPS: --");   }

  TFT_display.setTextColor(BLACK);
  TFT_display.setCursor(W/2 - 55, 6);
  TFT_display.print("SAT: ");
  TFT_display.print(sats);

  TFT_display.setCursor(W - 150, 6);
  TFT_display.print("BAT: ");
  TFT_display.print(batteryV, 1);
  TFT_display.print("V");
}

const char* turnTextFromErr(float errDegSigned){
  if (errDegSigned > 15) return "TURN RIGHT";
  if (errDegSigned < -15) return "TURN LEFT";
  return "STRAIGHT";
}

void drawBottomBar(float errDegSigned) {
  int y0 = H - BOTTOM_H;
  TFT_display.fillRect(0, y0, W, BOTTOM_H, GRAY);
  TFT_display.drawLine(0, y0, W, y0, BLACK);

  TFT_display.setTextSize(3);
  TFT_display.setTextColor(MAGENTA);
  TFT_display.setCursor(MARGIN, y0 + 10);
  TFT_display.print(turnTextFromErr(errDegSigned));

  TFT_display.setTextSize(2);
  TFT_display.setTextColor(BLACK);
  TFT_display.setCursor(W - 170, y0 + 18);
  TFT_display.print("ERR ");
  int errInt = (int)(errDegSigned >= 0 ? errDegSigned + 0.5f : errDegSigned - 0.5f);
  if (errInt >= 0) TFT_display.print("+");
  TFT_display.print(errInt);
  TFT_display.print((char)247);
}

// ---------- Arrow (FAST) ----------
// Single filled triangle arrow. 0° = up, 90° = right, 180° = down.
void drawArrowTriangle(float angleDeg, uint16_t color) {
  float rad = angleDeg * 3.1415926f / 180.0f;

  // local triangle points (pointing up at 0°)
  float L = ARROW_SIZE * 0.70f;      // length
  float Wd = ARROW_SIZE * 0.45f;     // width

  float x0 = 0.0f,    y0 = -L;       // tip
  float x1 = -Wd/2.f, y1 =  L*0.35f; // left base
  float x2 =  Wd/2.f, y2 =  L*0.35f; // right base

  auto RX = [&](float x, float y){ return x*cosf(rad) - y*sinf(rad); };
  auto RY = [&](float x, float y){ return x*sinf(rad) + y*cosf(rad); };

  int X0 = arrowCX + (int)RX(x0,y0), Y0 = arrowCY + (int)RY(x0,y0);
  int X1 = arrowCX + (int)RX(x1,y1), Y1 = arrowCY + (int)RY(x1,y1);
  int X2 = arrowCX + (int)RX(x2,y2), Y2 = arrowCY + (int)RY(x2,y2);

  TFT_display.fillTriangle(X0,Y0, X1,Y1, X2,Y2, color);
}

// Erase old arrow by drawing same triangle in WHITE (no rectangle wipe!)
void eraseArrow(float angleDeg) {
  drawArrowTriangle(angleDeg, WHITE);
}

// ---------- Distance box ----------
void drawDistanceBoxFrame() {
  TFT_display.drawRect(distBoxX, distBoxY, distBoxW, distBoxH, BLACK);
}

void drawDistanceValue(float meters) {
  // Update at 0.1 m resolution for “smooth”
  int tenth = (int)(meters * 10.0f + 0.5f);
  if (tenth == lastDistTenth) return;
  lastDistTenth = tenth;

  // Clear only INSIDE the box (small area)
  TFT_display.fillRect(distBoxX+2, distBoxY+2, distBoxW-4, distBoxH-4, WHITE);

  TFT_display.setTextColor(BLACK);
  TFT_display.setTextSize(6);

  char buf[16];
  // show 1 decimal for smoother feel: "51.2 m"
  snprintf(buf, sizeof(buf), "%.1f m", meters);

  // crude centering
  int len = (int)strlen(buf);
  int approxCharW = 6 * 6;
  int textW = len * approxCharW;
  int tx = distBoxX + (distBoxW - textW) / 2;
  int ty = distBoxY + 8;

  TFT_display.setCursor(tx, ty);
  TFT_display.print(buf);
}

// ---------- Layout ----------
void computeLayout() {
  W = TFT_display.width();
  H = TFT_display.height();

  mainTop = STATUS_H;
  mainBot = H - BOTTOM_H;
  mainH   = mainBot - mainTop;

  arrowCX = W / 2;
  arrowCY = mainTop + (int)(mainH * 0.40f);

  // distance box
  distBoxW = 300;
  distBoxH = 70;
  distBoxX = (W - distBoxW) / 2;
  distBoxY = mainTop + (int)(mainH * 0.68f);
}

// ---------- Smooth stepping (slew-limited) ----------
void stepAngleToward(float targetDeg) {
  // move smoothly using shortest path
  float d = angDiff(targetDeg, dispAngle);
  float maxStep = 3.0f;               // degrees per frame (~180°/sec at 60fps)
  d = clampf(d, -maxStep, maxStep);
  dispAngle = wrap360(dispAngle + d);
}

void stepDistToward(float targetM) {
  float d = targetM - dispDist;
  float maxStep = 0.08f;              // meters per frame (~4.8 m/sec at 60fps)
  d = clampf(d, -maxStep, maxStep);
  dispDist += d;
}

// ---------- Setup / Loop ----------
void setup() {
  Monitor.begin(9600);

  TFT_display.begin();
  TFT_display.setRotation(1);
  TFT_display.fillScreen(WHITE);

  computeLayout();

  // Static background (draw once)
  drawStatusBar(true, 7, 8.7f);
  drawBottomBar(0);
  drawDistanceBoxFrame();
  drawDistanceValue(dispDist);

  // First arrow draw
  drawArrowTriangle(dispAngle, MAGENTA);
  lastDrawAngle = dispAngle;
}

void loop() {
  uint32_t now = millis();

  // ===== DEMO TARGETS (smooth circle + distance increasing) =====
  // Replace these with real navigation values later.
  static float targetAngle = 0.0f;
  static float targetDist  = 41.0f;

  // Make it spin smoothly: ~1.5° per frame target change
  // (This simulates continuous user movement.)
  targetAngle = wrap360(targetAngle + 1.5f);

  // Make distance increase smoothly (getting farther)
  targetDist += 0.03f;           // ~1.8 m/sec
  if (targetDist > 51.0f) targetDist = 41.0f;

  // UI update @ ~60fps
  if (now - tUI >= UI_MS) {
    tUI = now;

    // Smoothly step displayed values toward targets
    stepAngleToward(targetAngle);
    stepDistToward(targetDist);

    // Redraw arrow ONLY if changed enough
    if (fabsf(angDiff(dispAngle, lastDrawAngle)) > 0.4f) {  // 0.4° threshold
      eraseArrow(lastDrawAngle);                 // erase old (no rectangle)
      drawArrowTriangle(dispAngle, MAGENTA);     // draw new
      lastDrawAngle = dispAngle;
    }

    // Distance smooth (0.1m changes)
    drawDistanceValue(dispDist);
  }

  // Status/bottom redraw slow
  if (now - tStatus >= STATUS_MS) {
    tStatus = now;

    // Demo values (replace later)
    static int sats = 7;
    sats = (sats % 12) + 1;
    bool gpsLock = (sats > 3);
    float batV = 8.7f;

    // For bottom bar “ERR”, use signed error relative to “forward”
    // Here we just show a synthetic signed error in [-180..180]
    float errSigned = wrap180(targetAngle); // demo only

    drawStatusBar(gpsLock, sats, batV);
    drawBottomBar(errSigned);
  }
}
