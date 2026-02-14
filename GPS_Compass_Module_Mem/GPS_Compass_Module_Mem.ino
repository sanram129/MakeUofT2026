#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LIS2MDL.h>
#include <EEPROM.h>

// Initialize Compass
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);

// Set up the Button Pin
const int buttonPin = A1;
int lastButtonState = HIGH; // We use HIGH because of INPUT_PULLUP

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  // Set A1 as an input with an internal pull-up resistor
  // The pin will read HIGH normally, and LOW when the button is pressed.
  pinMode(buttonPin, INPUT_PULLUP);

  Serial.println("\n--- EEPROM Compass Storage Test ---");

  // Start the Compass
  if (!mag.begin()) {
    Serial.println("Ooops, no LIS2MDL compass detected ... Check your wiring!");
    while (1);
  }

  // ==========================================
  // POWER-ON RECOVERY TEST
  // ==========================================
  float recoveredHeading = 0.0;
  
  // Read the 4 bytes starting at address 0 and put them into our float variable
  EEPROM.get(0, recoveredHeading);
  
  Serial.print("BOOT RECOVERY: Last saved heading was ");
  Serial.print(recoveredHeading);
  Serial.println(" degrees.");
  Serial.println("-----------------------------------");
  Serial.println("Rotate the compass, then press the button on A1 to save a new heading!");
}

void loop() {
  // Read the current state of the button
  int currentButtonState = digitalRead(buttonPin);

  // Check if the button was JUST pressed (transitioned from HIGH to LOW)
  if (lastButtonState == HIGH && currentButtonState == LOW) {
    
    // 1. Calculate the current compass heading
    sensors_event_t event;
    mag.getEvent(&event);
    float Pi = 3.14159;
    float currentHeading = (atan2(event.magnetic.y, event.magnetic.x) * 180) / Pi;
    if (currentHeading < 0) {
      currentHeading = 360 + currentHeading;
    }

    // 2. Save it to EEPROM
    EEPROM.put(0, currentHeading);
    
    Serial.println("\n=> BUTTON PRESSED!");
    Serial.print("=> Saved new heading: ");
    Serial.print(currentHeading);
    Serial.println(" degrees to EEPROM.");

    // 3. Retrieve it immediately to verify
    float verifyRetrieve = 0.0;
    EEPROM.get(0, verifyRetrieve);
    
    Serial.print("=> RETRIEVED FROM MEMORY: ");
    Serial.print(verifyRetrieve);
    Serial.println(" degrees.");
    Serial.println("\nTEST IT: Unplug the Arduino's USB cable, plug it back in, and see this exact value on boot!");
    Serial.println("-----------------------------------");

    // A short delay to debounce the button so it doesn't trigger 5 times in a single press
    delay(200); 
  }

  // Update the button state for the next loop
  lastButtonState = currentButtonState;
  
  // Small delay to keep the loop running smoothly
  delay(10); 
}