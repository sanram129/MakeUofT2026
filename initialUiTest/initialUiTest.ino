/*
  Pathfinder UI (Layout Option A): Big Arrow + Big Distance
  Target: Arduino UNO Q + DIYables_TFT_ILI9486_Shield

  Notes:
  - This code ONLY draws the UI with sample values.
  - Replace the sample values (gpsLock, sats, batteryV, distanceM, turnText, errDeg)
    with your real sensor/nav values later.
*/

#include <DIYables_TFT_Shield.h>
#include <Arduino_RouterBridge.h>

// Colors
#define WHITE     DIYables_TFT::colorRGB(255, 255, 255)
#define BLACK     DIYables_TFT::colorRGB(0, 0, 0)
#define MAGENTA   DIYables_TFT::colorRGB(255, 0, 255)
#define GREEN     DIYables_TFT::colorRGB(0, 170, 0)
#define RED       DIYables_TFT::colorRGB(200, 0, 0)
#define GRAY      DIYables_TFT::colorRGB(230, 230, 230)

DIYables_TFT_ILI9486_Shield TFT_display;

// ---------------- UI Geometry ----------------
static const int STATUS_H = 30;
static const int BOTTOM_H = 50;
static const int MARGIN   = 10;

// Simple arrow direction enum
enum ArrowDir { ARROW_STRAIGHT, ARROW_LEFT, ARROW_RIGHT };

// ---------------- Drawing Helpers ----------------

// Draw a filled, simple arrow using triangles/rectangles (no fancy fonts needed)
void drawArrow(int cx, int cy, int size, ArrowDir dir, uint16_t color) {
  // We draw a "UP" arrow in local coordinates then rotate by choosing point sets.
  // Use simple shapes: a shaft rectangle + a head triangle.
  int shaftW = size / 4;         // thickness
  int shaftH = (size * 3) / 5;   // length
  int headH  = (size * 2) / 5;   // head height
  int headW  = size;             // head width

  // For each direction we map to screen coords.
  // Local "UP" arrow parts:
  // Shaft: centered at (0, +headH/2) downwards
  // Head triangle: tip at (0, -size/2)
  // A) Straight (UP)
  if (dir == ARROW_STRAIGHT) {
    // Shaft rectangle
    int x0 = cx - shaftW / 2;
    int y0 = cy - (shaftH / 2) + headH / 2;
    TFT_display.fillRect(x0, y0, shaftW, shaftH, color);

    // Head triangle
    int tipX = cx;
    int tipY = cy - size / 2;
    int leftX = cx - headW / 2;
    int leftY = cy - size / 2 + headH;
    int rightX = cx + headW / 2;
    int rightY = cy - size / 2 + headH;

    TFT_display.fillTriangle(tipX, tipY, leftX, leftY, rightX, rightY, color);
  }
  // B) Left (arrow points LEFT)
  else if (dir == ARROW_LEFT) {
    // Shaft rectangle (horizontal)
    int x0 = cx - (shaftH / 2) - headH / 2;
    int y0 = cy - shaftW / 2;
    TFT_display.fillRect(x0, y0, shaftH, shaftW, color);

    // Head triangle pointing left
    int tipX = cx - size / 2;
    int tipY = cy;
    int topX = cx - size / 2 + headH;
    int topY = cy - headW / 2;
    int botX = cx - size / 2 + headH;
    int botY = cy + headW / 2;

    TFT_display.fillTriangle(tipX, tipY, topX, topY, botX, botY, color);
  }
  // C) Right (arrow points RIGHT)
  else {
    // Shaft rectangle (horizontal)
    int x0 = cx - (shaftH / 2) + headH / 2;
    int y0 = cy - shaftW / 2;
    TFT_display.fillRect(x0, y0, shaftH, shaftW, color);

    // Head triangle pointing right
    int tipX = cx + size / 2;
    int tipY = cy;
    int topX = cx + size / 2 - headH;
    int topY = cy - headW / 2;
    int botX = cx + size / 2 - headH;
    int botY = cy + headW / 2;

    TFT_display.fillTriangle(tipX, tipY, topX, topY, botX, botY, color);
  }
}

void drawStatusBar(int w, bool gpsLock, int sats, float batteryV) {
  // Background
  TFT_display.fillRect(0, 0, w, STATUS_H, GRAY);
  TFT_display.drawLine(0, STATUS_H - 1, w, STATUS_H - 1, BLACK);

  // Left: GPS
  TFT_display.setTextSize(2);
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
  TFT_display.print((char)247); // degree symbol (often works as 247)
}

void drawMainArea(int w, int h, ArrowDir dir, int distanceM) {
  int yTop = STATUS_H;
  int yBot = h - BOTTOM_H;
  int areaH = yBot - yTop;

  // Clear main area
  TFT_display.fillRect(0, yTop, w, areaH, WHITE);

  // Arrow center
  int cx = w / 2;
  int cy = yTop + areaH / 2 - 40;

  // Draw arrow
  drawArrow(cx, cy, 180, dir, MAGENTA);

  // Distance big text under arrow
  TFT_display.setTextColor(BLACK);
  TFT_display.setTextSize(6);

  // Rough centering (monospace-ish estimate): each char ~6*textSize pixels wide.
  // We'll compute a simple width estimate for "### m"
  char buf[16];
  snprintf(buf, sizeof(buf), "%d m", distanceM);
  int len = (int)strlen(buf);
  int approxCharW = 6 * 6; // base font ~6px wide * textSize(6)
  int textW = len * approxCharW;

  int tx = (w - textW) / 2;
  int ty = cy + 110;

  TFT_display.setCursor(tx, ty);
  TFT_display.print(buf);
}

// Draw a complete screen using the Option A layout
void drawPathfinderScreen(int w, int h,
                          bool gpsLock, int sats, float batteryV,
                          ArrowDir dir, int distanceM,
                          const char* turnText, int errDeg) {
  drawStatusBar(w, gpsLock, sats, batteryV);
  drawMainArea(w, h, dir, distanceM);
  drawBottomBar(w, h, turnText, errDeg);
}

// ---------------- Arduino Setup/Loop ----------------

void setup() {
  Monitor.begin(9600);
  Monitor.println("Pathfinder UI demo (Layout A)");

  TFT_display.begin();
  TFT_display.setRotation(1);   // landscape
  TFT_display.fillScreen(WHITE);

  // If your library supports it, you can set text background:
  // TFT_display.setTextColor(color, bgColor);
  // We'll just clear regions before drawing.
}

void loop() {
  // ----- SAMPLE VALUES (replace with real data later) -----
  static bool gpsLock = true;
  static int sats = 10;
  static float batteryV = 8.7;

  static int distanceM = 42;
  static int errDeg = +35;

  // Choose arrow direction based on error
  ArrowDir dir;
  const char* turnText;

  if (errDeg > 15) {
    dir = ARROW_RIGHT;
    turnText = "TURN RIGHT";
  } else if (errDeg < -15) {
    dir = ARROW_LEFT;
    turnText = "TURN LEFT";
  } else {
    dir = ARROW_STRAIGHT;
    turnText = "STRAIGHT";
  }

  // Screen size: most ILI9486 shields are 480x320 in landscape
  int w = TFT_display.width();
  int h = TFT_display.height();

  drawPathfinderScreen(w, h, gpsLock, sats, batteryV, dir, distanceM, turnText, errDeg);

  // Animate sample values so you can see updates
  errDeg += 10;
  if (errDeg > 60) errDeg = -60;

  distanceM -= 1;
  if (distanceM < 5) distanceM = 42;

  sats = (sats % 12) + 1;
  gpsLock = (sats > 3);

  delay(500);
}

