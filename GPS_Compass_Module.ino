#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_Accel.h>
#include <Adafruit_LIS2MDL.h>
#include "DFRobot_GNSSAndRTC.h"

// ==========================================
// 1. SENSOR INITIALIZATION
// ==========================================

// Initialize Compass (Magnetometer)
Adafruit_LIS2MDL mag = Adafruit_LIS2MDL(12345);

// Initialize Accelerometer
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);

// Initialize GPS (using I2C communication)
#define I2C_COMMUNICATION 
DFRobot_GNSSAndRTC_I2C gnss(&Wire, MODULE_I2C_ADDRESS);

void setup(void) {
  // Set unified baud rate
  Serial.begin(115200);
  
  // Wait for serial console to open
  while (!Serial) delay(10); 
  
  Serial.println("========================================");
  Serial.println("  Survival Pathway Finder Initializing  ");
  Serial.println("========================================");

  // --- Start Magnetometer ---
  if (!mag.begin()) {
    Serial.println("ERROR: No LIS2MDL Compass detected... Check wiring!");
    while (1);
  }
  Serial.println("Compass Status:   OK");

  // --- Start Accelerometer ---
  if (!accel.begin()) {
    Serial.println("ERROR: No LSM303 Accelerometer detected... Check wiring!");
    while (1);
  }
  accel.setRange(LSM303_RANGE_4G);
  accel.setMode(LSM303_MODE_NORMAL);
  Serial.println("Accel Status:     OK");

  // --- Start GPS ---
  if (!gnss.begin()) {
    Serial.println("ERROR: No DFRobot GNSS detected... Check wiring!");
    delay(1000);
  }
  gnss.enablePower(); 
  gnss.setGnss(gnss.eGPS_BeiDou_GLONASS); 
  Serial.println("GPS Status:       OK (Waiting for lock)");
  
  Serial.println("========================================\n");
  delay(2000);
}

void loop(void) {
  
  Serial.println("--- Current Sensor Readings ---");

  // ==========================================
  // 2. COMPASS DATA (Angle to North)
  // ==========================================
  sensors_event_t mag_event;
  mag.getEvent(&mag_event);
  
  float Pi = 3.14159;
  float heading = (atan2(mag_event.magnetic.y, mag_event.magnetic.x) * 180) / Pi;
  if (heading < 0) {
    heading = 360 + heading;
  }
  
  Serial.print("Compass Heading:  ");
  Serial.print(heading);
  Serial.println(" degrees");


  // ==========================================
  // 3. ACCELEROMETER DATA (X and Y only)
  // ==========================================
  sensors_event_t accel_event;
  accel.getEvent(&accel_event);

  Serial.print("Tilt (X/Y):       X: ");
  Serial.print(accel_event.acceleration.x);
  Serial.print(" m/s^2  |  Y: ");
  Serial.print(accel_event.acceleration.y);
  Serial.println(" m/s^2");


  // ==========================================
  // 4. GPS POSITIONAL DATA
  // ==========================================
  DFRobot_GNSSAndRTC::sLonLat_t lat = gnss.getLat();
  DFRobot_GNSSAndRTC::sLonLat_t lon = gnss.getLon();
  double alt = gnss.getAlt();
  uint8_t satellites = gnss.getNumSatUsed();

  Serial.print("Satellites:       ");
  Serial.println(satellites);
  
  if (satellites > 0) {
    Serial.print("Latitude:         ");
    Serial.print(lat.latitudeDegree, 6);
    Serial.println((char)lat.latDirection);
    
    Serial.print("Longitude:        ");
    Serial.print(lon.lonitudeDegree, 6);
    Serial.println((char)lon.lonDirection);
    
    Serial.print("Altitude:         ");
    Serial.print(alt);
    Serial.println(" m");
  } else {
    Serial.println("GPS Location:     [Searching for Satellites...]");
  }


  // ==========================================
  // 5. GPS TIME & DATE (Converted to EST)
  // ==========================================
  DFRobot_GNSSAndRTC::sTim_t utc = gnss.getUTC();
  DFRobot_GNSSAndRTC::sTim_t date = gnss.getDate();

  // GPS defaults to year 0 or 2000 before it gets a valid satellite time lock
  if (date.year > 2000) { 
    int est_hour = utc.hour - 5; // EST is UTC-5
    int est_date = date.date;
    int est_month = date.month;
    int est_year = date.year;

    // Handle rolling back the day if subtracting 5 hours pushed us past midnight
    if (est_hour < 0) {
      est_hour += 24;
      est_date -= 1;
      
      // Handle rolling back the month
      if (est_date < 1) {
        est_month -= 1;
        if (est_month < 1) {
          est_month = 12;
          est_year -= 1;
        }
        // Figure out how many days were in the previous month
        uint8_t daysInMonth[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
        // Leap year check for February
        if (est_year % 4 == 0 && (est_year % 100 != 0 || est_year % 400 == 0)) {
            daysInMonth[2] = 29;
        }
        est_date = daysInMonth[est_month];
      }
    }

    // Print Date
    Serial.print("Date (EST):       ");
    Serial.print(est_year); Serial.print("/"); 
    if(est_month < 10) Serial.print("0"); // Zero padding
    Serial.print(est_month); Serial.print("/"); 
    if(est_date < 10) Serial.print("0");
    Serial.println(est_date);
    
    // Print Time
    Serial.print("Time (EST):       ");
    if(est_hour < 10) Serial.print("0");
    Serial.print(est_hour); Serial.print(":");
    if(utc.minute < 10) Serial.print("0");
    Serial.print(utc.minute); Serial.print(":");
    if(utc.second < 10) Serial.print("0");
    Serial.println(utc.second);
    
  } else {
    Serial.println("GPS Time:         [Searching for Satellites...]");
  }

  Serial.println("\n");
  
  // Delay 1 second before next full reading
  delay(1000); 
}