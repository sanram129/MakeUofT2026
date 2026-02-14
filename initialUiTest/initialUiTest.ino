/*
  Pathfinder UI: Smooth 0–360 arrow + smooth distance update
  Board: Arduino UNO Q
  Display: DIYables 3.5" ILI9486 Shield (DIYables_TFT_Shield + Adafruit_GFX)

  Key idea:
  - Draw static UI once.
  - For animation, update ONLY:
      (1) erase old arrow by drawing it in WHITE
      (2) draw new arrow in MAGENTA
      (3) re-print distance using setTextColor(fg, bg) so no rectangle clear is needed
*/

#include <DIYables_TFT_Shield.h>
#include <Arduino_RouterBridge.h>
#include <math.h>

// Colors
#define WHITE     DIYables_TFT::colorRGB(255, 255, 255)
#define BLACK     DIYables_TFT::colorRGB(0, 0, 0)
#define MAGENTA   DIYables_TFT::colorRGB(255, 0, 255)
#define GREEN     DIYables_TFT::colorRGB(0, 170, 0)
#define RED       DIYables_TFT::colorRGB(200, 0, 0)
#define GRAY      DIYables_TFT::colorRGB(230, 230, 230)

DIYables_TFT_ILI9486_Shield TFT_display;

// UI geometry
static const int STATUS_H = 28;
static const int BOTTOM_H = 44;
static const int MARGIN   = 8;

// Arrow geometry (smaller than before)
static const int ARROW_LEN = 55;   // tip length
static const int ARROW_W   = 34;   // base width

// Positions (computed after begin)
int W, H;
int mainTop, mainBot, mainH;
int arrowCX, arrowCY;
int distBoxX, distBoxY, distBoxW, distBoxH;
int distTextX, distTextY;

// State for smooth animation
float shownAngleDeg = 0.0f;     // what we draw
float targetAngleDeg = 0.0f;    // what nav wants
float prevDrawnAngle = NAN;

float shownDistM = 40.0f;
float targetDistM = 40.0f;

// -------- helpers --------
static float wrapDeltaDeg(float target, float current) {
  // shortest signed delta in [-180, +180]
  float d = fmodf(target - current + 540.0f, 360.0f) - 180.0f;
  return d;
}

static void computeLayout() {
  W = TFT_display.width();
  H = TFT_display.height();

  mainTop = STATUS_H;
  mainBot = H - BOTTOM_H;
  mainH   = mainBot - mainTop;

  arrowCX = W / 2;
  arrowCY = mainTop + (int)(mainH * 0.40f);

  // Distance box below arrow
  distBoxW = (int)(W * 0.70f);
  distBoxH = 60;
  distBoxX = (W - distBoxW) / 2;
  distBoxY = arrowCY + 35;

  // Distance text inside the box
  distTextX = distBoxX + 25;
  distTextY = distBoxY + 18;
}

static void drawStaticUI() {
  // Background
  TFT_display.fillScreen(WHITE);

  // Status bar
  TFT_display.fillRect(0, 0, W, STATUS_H, GRAY);
  TFT_display.drawLine(0, STATUS_H - 1, W, STATUS_H - 1, BLACK);

  // Bottom bar
  TFT_display.fillRect(0, H - BOTTOM_H, W, BOTTOM_H, GRAY);
  TFT_display.drawLine(0, H - BOTTOM_H, W, H - BOTTOM_H, BLACK);

  // Distance box border (static)
  TFT_display.drawRect(distBoxX, distBoxY, distBoxW, distBoxH, BLACK);
}

static void updateStatus(bool gpsLock, int sats, float battV) {
  // Print over same locations with background to erase old text
  TFT_display.setTextSize(2);

  // GPS left
  TFT_display.setCursor(MARGIN, 6);
  TFT_display.setTextColor(gpsLock ? GREEN : RED, GRAY);
  TFT_display.print("GPS: ");
  TFT_display.print(gpsLock ? "LOCK" : "--  "); // pad a bit

  // SAT middle
  TFT_display.setCursor(W / 2 - 40, 6);
  TFT_display.setTextColor(BLACK, GRAY);
  TFT_display.print("SAT:");
  TFT_display.print(sats);
  TFT_display.print("  "); // erase leftovers

  // BAT right
  TFT_display.setCursor(W - 140, 6);
  TFT_display.setTextColor(BLACK, GRAY);
  TFT_display.print("BAT:");
  TFT_display.print(battV, 1);
  TFT_display.print("V ");
}

static void updateBottom(const char* turnText, int errDeg) {
  int y0 = H - BOTTOM_H;

  // Turn text (left)
  TFT_display.setTextSize(3);
  TFT_display.setCursor(MARGIN, y0 + 10);
  TFT_display.setTextColor(MAGENTA, GRAY);
  TFT_display.print(turnText);
  TFT_display.print("   "); // erase leftovers

  // Error (right)
  TFT_display.setTextSize(2);
  TFT_display.setCursor(W - 150, y0 + 14);
  TFT_display.setTextColor(BLACK, GRAY);
  TFT_display.print("ERR ");
  if (errDeg >= 0) TFT_display.print("+");
  TFT_display.print(errDeg);
  TFT_display.print((char)247);
  TFT_display.print("   ");
}

static void drawArrowTriangle(float angleDeg, uint16_t color) {
  // Define 0° = UP, 90° = RIGHT, clockwise positive
  float th = angleDeg * (3.1415926f / 180.0f);

  float dx = sinf(th);
  float dy = -cosf(th);

  float px = cosf(th);
  float py = sinf(th);

  // tip point
  int xTip = (int)(arrowCX + dx * ARROW_LEN);
  int yTip = (int)(arrowCY + dy * ARROW_LEN);

  // base center (a bit behind center)
  float back = ARROW_LEN * 0.45f;
  int xBaseC = (int)(arrowCX - dx * back);
  int yBaseC = (int)(arrowCY - dy * back);

  int halfW = ARROW_W / 2;
  int xL = (int)(xBaseC - px * halfW);
  int yL = (int)(yBaseC - py * halfW);
  int xR = (int)(xBaseC + px * halfW);
  int yR = (int)(yBaseC + py * halfW);

  TFT_display.fillTriangle(xTip, yTip, xL, yL, xR, yR, color);

  // small tail line to make it look more like an arrow (optional)
  int xTail = (int)(arrowCX - dx * (ARROW_LEN * 0.35f));
  int yTail = (int)(arrowCY - dy * (ARROW_LEN * 0.35f));
  TFT_display.drawLine(xBaseC, yBaseC, xTail, yTail, color);
}

static void updateArrowSmooth(float newAngleDeg) {
  // erase old arrow by re-drawing it in WHITE (tiny erase area, no big rectangle)
  if (!isnan(prevDrawnAngle)) {
    drawArrowTriangle(prevDrawnAngle, WHITE);
  }
  drawArrowTriangle(newAngleDeg, MAGENTA);
  prevDrawnAngle = newAngleDeg;
}

static void updateDistance(float distM) {
  // Print with background WHITE so digits overwrite cleanly (no fillRect wipe)
  TFT_display.setTextSize(5);
  TFT_display.setTextColor(BLACK, WHITE);
  TFT_display.setCursor(distTextX, distTextY);

  // Fixed-width string helps erase old longer values
  char buf[20];
  // Example: " 44.8 m" (padded)
  snprintf(buf, sizeof(buf), "%6.1f m", distM);
  TFT_display.print(buf);
}

// -------- Arduino --------
void setup() {
  Monitor.begin(9600);
  TFT_display.begin();
  TFT_display.setRotation(1);

  computeLayout();
  drawStaticUI();

  // Initial labels
  updateStatus(true, 7, 8.7);
  updateBottom("TURN RIGHT", +63);
  updateDistance(shownDistM);
}

void loop() {
  // DEMO target motion:
  // - arrow rotates continuously
  // - distance increases
  targetAngleDeg += 6.0f;   // change this for speed
  if (targetAngleDeg >= 360.0f) targetAngleDeg -= 360.0f;

  targetDistM += 0.08f;     // change this for speed
  if (targetDistM > 99.9f) targetDistM = 40.0f;

  // Smooth the displayed values (low-pass filter)
  // Bigger alpha = snappier, smaller alpha = smoother
  const float alphaAngle = 0.22f;
  const float alphaDist  = 0.18f;

  float dAng = wrapDeltaDeg(targetAngleDeg, shownAngleDeg);
  shownAngleDeg += alphaAngle * dAng;
  shownAngleDeg = fmodf(shownAngleDeg + 360.0f, 360.0f);

  shownDistM += alphaDist * (targetDistM - shownDistM);

  // Update only what changed (and only at a controlled frame rate)
  static uint32_t lastFrame = 0;
  const uint32_t FRAME_MS = 30; // ~33 FPS. Raise to 40–60ms if too heavy.

  uint32_t now = millis();
  if (now - lastFrame >= FRAME_MS) {
    lastFrame = now;

    updateArrowSmooth(shownAngleDeg);
    updateDistance(shownDistM);
  }

  // (Optional) update bars slower, like 2 Hz, not every frame:
  // static uint32_t lastBars = 0;
  // if (now - lastBars > 500) { lastBars = now; updateStatus(...); updateBottom(...); }
}
