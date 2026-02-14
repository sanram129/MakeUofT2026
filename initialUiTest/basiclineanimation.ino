#include <DIYables_TFT_Shield.h>
#include <Arduino_RouterBridge.h>
#include <math.h>
#include <stdio.h>

DIYables_TFT_ILI9486_Shield TFT;

// Colors
#define WHITE   DIYables_TFT::colorRGB(255, 255, 255)
#define BLACK   DIYables_TFT::colorRGB(0, 0, 0)
#define MAGENTA DIYables_TFT::colorRGB(255, 0, 255)
#define GREEN   DIYables_TFT::colorRGB(0, 170, 0)
#define RED     DIYables_TFT::colorRGB(200, 0, 0)
#define GRAY    DIYables_TFT::colorRGB(230, 230, 230)

static const int STATUS_H = 28;
static const int MARGIN   = 8;

float prevAngle = NAN;

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
  TFT.print(gpsLock ? "LOCK" : "--  "); // pad to overwrite

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

// ---------------- Arrow Drawing ----------------
void drawThickLine(int x0, int y0, int x1, int y1, int thickness, uint16_t color) {
  // Simple thick line: multiple parallel lines (fast-ish)
  for (int i = -thickness/2; i <= thickness/2; i++) {
    TFT.drawLine(x0 + i, y0, x1 + i, y1, color);
    TFT.drawLine(x0, y0 + i, x1, y1 + i, color);
  }
}

void drawArrow(float angleDeg, uint16_t color) {
  int W = TFT.width();
  int H = TFT.height();

  // Center arrow in MAIN AREA (below status bar)
  int cx = W / 2;
  int cy = (STATUS_H + H) / 2;   // center of remaining area

  // Arrow shape (short + thick)
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

void setup() {
  Monitor.begin(9600);

  TFT.begin();
  TFT.setRotation(1);
  TFT.fillScreen(WHITE);

  // Static top panel
  drawStatusBarFrame();

  // Initial values
  updateStatusBar(false, 0, 0.0f);

  // Center dot (optional) in main area
  int cx = TFT.width() / 2;
  int cy = (STATUS_H + TFT.height()) / 2;
  TFT.fillCircle(cx, cy, 3, BLACK);
}

void loop() {
  static float angle = 0.0f;

  // ---- Demo status values (replace later) ----
  static int sats = 0;
  static bool gpsLock = false;
  static float battV = 8.7f;

  // Update status slower than arrow
  static uint32_t lastStatus = 0;
  if (millis() - lastStatus > 500) {
    lastStatus = millis();

    sats = (sats % 12) + 1;
    gpsLock = (sats > 3);

    updateStatusBar(gpsLock, sats, battV);
  }

  // Erase previous arrow
  if (!isnan(prevAngle)) {
    drawArrow(prevAngle, WHITE);
  }

  // Draw new arrow
  drawArrow(angle, MAGENTA);
  prevAngle = angle;

  // Advance angle
  angle += 3.0f;
  if (angle >= 360.0f) angle -= 360.0f;

  delay(25);
}
