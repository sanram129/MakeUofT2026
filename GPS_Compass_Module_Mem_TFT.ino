#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <MCUFRIEND_kbv.h>
#include <TouchScreen.h>

// ==========================================
// 1. HARDWARE SETUP
// ==========================================

// Initialize Compass
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);

// Initialize Display
MCUFRIEND_kbv tft;

// Touchscreen pins (Standard for 3.5" UNO/Mega Shields)
#define YP A3  // must be an analog pin
#define XM A2  // must be an analog pin
#define YM 9   // can be a digital pin
#define XP 8   // can be a digital pin

// Touch pressure thresholds to filter out noise
#define MINPRESSURE 100
#define MAXPRESSURE 1000

// Initialize Touchscreen
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);

// Basic 16-bit Color definitions
#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF

// Track touch state for edge detection (pressing vs holding)
bool wasTouched = false;

// ==========================================
// 2. HELPER FUNCTION: UPDATE SCREEN
// ==========================================
// We use this to draw the text so we don't repeat code in setup() and loop()
void updateScreenText(float heading, bool isNewSave) {
  // Draw a black rectangle over the old text area to clear it
  tft.fillRect(0, 40, 320, 100, BLACK); 
  
  // Draw the label
  tft.setCursor(20, 50);
  tft.setTextColor(isNewSave ? GREEN : CYAN);
  tft.setTextSize(2);
  tft.print(isNewSave ? "Successfully Saved:" : "Boot Recovery:");
  
  // Draw the heading number nice and large
  tft.setCursor(20, 90);
  tft.setTextColor(WHITE);
  tft.setTextSize(4);
  tft.print(heading);
  tft.print(" deg");
}

// ==========================================
// 3. MAIN SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  // --- Start Compass ---
  if (!mag.begin()) {
    Serial.println("Ooops, no LIS2MDL compass detected ... Check wiring!");
    while (1);
  }

  // --- Start Display ---
  tft.reset();
  uint16_t identifier = tft.readID();
  if (identifier == 0xD3D3) identifier = 0x9486; // Common fallback for some shields
  tft.begin(identifier);
  tft.setRotation(1); // Set to Landscape mode
  tft.fillScreen(BLACK);

  // Draw the "Save" Button UI
  // x=60, y=160, width=200, height=60, corner_radius=10, color=RED
  tft.fillRoundRect(60, 160, 200, 60, 10, RED); 
  tft.setCursor(85, 180);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  tft.print("TAP TO SAVE");

  // --- Power-on Recovery ---
  float recoveredHeading = 0.0;
  EEPROM.get(0, recoveredHeading);
  
  // Print recovered value to the LCD
  updateScreenText(recoveredHeading, false);
}

// ==========================================
// 4. MAIN LOOP
// ==========================================
void loop() {
  // 1. Read the touch coordinates and pressure
  TSPoint p = ts.getPoint();
  
  // IMPORTANT: The touch library changes the pin modes of XM and YP. 
  // We MUST set them back to OUTPUT so the TFT can draw properly afterwards.
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);

  // 2. Determine if the screen is currently being pressed
  bool isTouched = (p.z > MINPRESSURE && p.z < MAXPRESSURE);

  // 3. Check for a NEW touch (transition from untouched to touched)
  if (isTouched && !wasTouched) {
    
    // Calculate the current compass heading
    sensors_event_t event;
    mag.getEvent(&event);
    float Pi = 3.14159;
    float currentHeading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;
    if (currentHeading < 0) {
      currentHeading = 360 + currentHeading;
    }

    // Save to EEPROM
    EEPROM.put(0, currentHeading);

    // Retrieve immediately to verify
    float verifyRetrieve = 0.0;
    EEPROM.get(0, verifyRetrieve);

    // Update the LCD with the new verified heading
    updateScreenText(verifyRetrieve, true);

    // --- Button Visual Feedback ---
    // Flash the button GREEN to let the user know it worked
    tft.fillRoundRect(60, 160, 200, 60, 10, GREEN);
    tft.setCursor(95, 180);
    tft.print(" SAVED! ");
    
    delay(500); // Leave green for half a second
    
    // Re-draw the button RED for the next press
    tft.fillRoundRect(60, 160, 200, 60, 10, RED);
    tft.setCursor(85, 180);
    tft.print("TAP TO SAVE");
  }

  // Update touch state for the next loop
  wasTouched = isTouched;
  
  delay(20); // Small delay to keep things running smoothly
}