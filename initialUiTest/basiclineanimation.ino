#include <DIYables_TFT_Shield.h>
#include <Arduino_RouterBridge.h>
#include <math.h>

DIYables_TFT_ILI9486_Shield TFT;

#define WHITE   DIYables_TFT::colorRGB(255, 255, 255)
#define BLACK   DIYables_TFT::colorRGB(0, 0, 0)
#define MAGENTA DIYables_TFT::colorRGB(255, 0, 255)

float prevAngle = NAN;

void drawThickLine(int x0, int y0, int x1, int y1, int thickness, uint16_t color) {
  // Simple thick line: draw several parallel lines
  // Works well for small thickness (e.g., 3..10)
  for (int i = -thickness/2; i <= thickness/2; i++) {
    // Offset both x and y to make it "thick" in a simple way
    TFT.drawLine(x0 + i, y0, x1 + i, y1, color);
    TFT.drawLine(x0, y0 + i, x1, y1 + i, color);
  }
}

void drawArrow(float angleDeg, uint16_t color) {
  int W = TFT.width();
  int H = TFT.height();
  int cx = W / 2;
  int cy = H / 2;

  // --- Make it shorter (middle of screen) ---
  int shaftLen = 70;     // overall arrow length (shorter)
  int shaftLenBack = 10; // small back extension
  int thickness = 6;     // thicker shaft

  // Arrowhead size
  int headLen = 22;
  int headW   = 18;

  float th = angleDeg * (3.1415926f / 180.0f);

  // Direction unit vector
  float dx = cosf(th);
  float dy = sinf(th);

  // Perpendicular unit vector
  float px = -dy;
  float py = dx;

  // Tip of arrow
  int xTip = cx + (int)(dx * shaftLen);
  int yTip = cy + (int)(dy * shaftLen);

  // Shaft ends a bit before tip
  int xShaftEnd = cx + (int)(dx * (shaftLen - headLen));
  int yShaftEnd = cy + (int)(dy * (shaftLen - headLen));

  // Shaft start slightly behind center (optional)
  int xShaftStart = cx - (int)(dx * shaftLenBack);
  int yShaftStart = cy - (int)(dy * shaftLenBack);

  // Draw thick shaft
  drawThickLine(xShaftStart, yShaftStart, xShaftEnd, yShaftEnd, thickness, color);

  // Arrowhead triangle
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

  // center dot
  TFT.fillCircle(TFT.width()/2, TFT.height()/2, 3, BLACK);
}

void loop() {
  static float angle = 0.0f;

  // erase previous
  if (!isnan(prevAngle)) {
    drawArrow(prevAngle, WHITE);
  }

  // draw current
  drawArrow(angle, MAGENTA);
  prevAngle = angle;

  // update angle
  angle += 3.0f;          // speed (try 1..6)
  if (angle >= 360.0f) angle -= 360.0f;

  delay(25);              // smoothness (try 15..40)
}
