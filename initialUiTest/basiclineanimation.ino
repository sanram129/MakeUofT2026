#include <DIYables_TFT_Shield.h>
#include <Arduino_RouterBridge.h>
#include <math.h>

DIYables_TFT_ILI9486_Shield TFT;

#define WHITE   DIYables_TFT::colorRGB(255, 255, 255)
#define BLACK   DIYables_TFT::colorRGB(0, 0, 0)
#define MAGENTA DIYables_TFT::colorRGB(255, 0, 255)

float prevAngle = NAN;

void drawLineAtAngle(float angleDeg, uint16_t color) {
  int W = TFT.width();
  int H = TFT.height();

  int cx = W / 2;
  int cy = H / 2;

  int r = (min(W, H) / 2) - 20;  // radius / length

  float th = angleDeg * (3.1415926f / 180.0f);

  int x2 = cx + (int)(r * cosf(th));
  int y2 = cy + (int)(r * sinf(th));

  TFT.drawLine(cx, cy, x2, y2, color);
}

void setup() {
  Monitor.begin(9600);

  TFT.begin();
  TFT.setRotation(1);
  TFT.fillScreen(WHITE);

  // optional center mark
  int cx = TFT.width() / 2;
  int cy = TFT.height() / 2;
  TFT.fillCircle(cx, cy, 3, BLACK);
}

void loop() {
  static float angle = 0.0f;

  // erase old line
  if (!isnan(prevAngle)) {
    drawLineAtAngle(prevAngle, WHITE);
  }

  // draw new line
  drawLineAtAngle(angle, MAGENTA);
  prevAngle = angle;

  // advance angle
  angle += 3.0f;         // change this for speed (try 1.0..6.0)
  if (angle >= 360.0f) angle -= 360.0f;

  delay(25);             // change this for smoothness (try 15..35)
}
