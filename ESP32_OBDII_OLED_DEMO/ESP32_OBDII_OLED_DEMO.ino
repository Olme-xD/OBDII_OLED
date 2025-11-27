/*
* ESP32_OBDII_DEMO.ino
* Based on code by Olme-xD (Made by AI)
* * DEMO MODE: Simulates vehicle data to test OLED layout and Menus.
* No Bluetooth/OBDII connection required.
*/

// Libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "FreeSansBold40pt7b.h"
#include "FreeSansBold14pt7b.h"
#include "FreeSansBold12pt7b.h"
#include "FreeSansBold9pt7b.h"

// OLED Screen Settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define I2C_SDA 21
#define I2C_SCL 22

// Switch Pin Set
#define SWITCH 12

// Instances
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Global Simulated Variables
float global_kph = 0.0;
float global_maf = 0.0;
float global_rpm = 0.0;
float global_load = 0.0;
float global_fuel = 100.0;
double global_totalDistance = 0.0;
double global_totalFuel = 0.0;
char global_dtcString[50] = "P0300 Random Misfire\nP0171 System Lean";

// Helper: Timer Transform (Formats millis to HH:MM:SS)
String timerTransform(uint32_t millisTime) {
  String finalTimer = "";
  uint16_t seconds = millisTime / 1000;
  uint16_t hours = seconds / 3600;
  seconds %= 3600;
  uint16_t minutes = seconds / 60;
  seconds %= 60;

  if (hours >= 1) {
    if (hours < 10) finalTimer += '0';
    finalTimer += String(hours) + ":";
  }
  if (minutes < 10) finalTimer += '0';
  finalTimer += String(minutes) + ":";
  if (seconds < 10) finalTimer += '0';
  finalTimer += String(seconds);

  return finalTimer;
}

// Helper: Simulate Vehicle Data (Replaces OBD Task)
void simulateVehicleData() {
  static float sim_angle = 0.0;
  
  // Create a sine wave pattern for speed (0 to 120 kph)
  sim_angle += 0.05;
  if(sim_angle > 6.28) sim_angle = 0;
  
  // Fake Physics
  global_kph = (sin(sim_angle) + 1) * 60; // 0 to 120 KPH
  global_rpm = (global_kph * 40) + 800;   // RPM follows speed
  global_maf = global_rpm / 150.0;        // MAF follows RPM
  global_load = (sin(sim_angle * 2) + 1) * 50; // Load varies randomly
  
  // Simulate consumption
  if(global_kph > 0) {
    global_totalDistance += (global_kph / 3600.0) * 0.1; // Add distance
    global_totalFuel += (global_maf * 0.001); // Add fuel
    global_fuel -= 0.001; // Drain tank
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(SWITCH, INPUT_PULLUP);

  // Initialize OLED Display
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 init failed"));
    for(;;);
  }

  // Initial Settings
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(0xCF); 
  
  // Intro Screen
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OBDII DEMO MODE");
  display.println("Simulation Only\n");
  display.println("---------------------");
  display.print("Starting...");
  display.display();
  delay(2000);
}

void loop() {
  static int local_mode = 1;
  static int tapCounter = 0;
  static uint32_t lastTapTime = 0;
  static int lastButtonReading = LOW;
  static int currentButtonState = LOW;
  static uint32_t lastDebounceTime = 0;
  
  // 1. UPDATE SIMULATION
  simulateVehicleData();

  // 2. READ BUTTON (Mode Switch Logic)
  int reading = digitalRead(SWITCH);
  if (reading != lastButtonReading) lastDebounceTime = millis();
  lastButtonReading = reading; 

  if ((millis() - lastDebounceTime) > 30) {
    if (reading != currentButtonState) {
      currentButtonState = reading;
      if (currentButtonState == HIGH) {
        tapCounter++;
        if (tapCounter > 6) tapCounter = 1; // Limit to 6 modes for demo
        lastTapTime = millis();

        // UI Feedback
        display.clearDisplay();
        display.setFont(&FreeSansBold12pt7b);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(15, 40);
        display.print("MODE: ");
        display.print(tapCounter);
        display.display();
      }
    }
  }
  
  // Wait for button timeout to switch mode
  if (tapCounter > 0 && (millis() - lastTapTime) > 1000) {
    local_mode = tapCounter;
    tapCounter = 0; 
  }
  if (tapCounter > 0) return; 

  // 3. DRAW DISPLAY
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setFont(NULL);

  switch(local_mode) {
    case 1: { // MAIN DASH
      float_t inst_mpg = 0.0;
      if (global_kph > 0 && global_maf > 0) {
         inst_mpg = (14.7 * 6.17 * 4.54 * global_kph * 0.621371) / (3600 * global_maf / 100);
      }
      
      // Instant MPG
      display.setFont(&FreeSansBold40pt7b);
      display.setCursor(0, 60);
      if (inst_mpg > 0 && inst_mpg < 99) display.print(inst_mpg, 0);
      else display.print("--");

      // Avg MPG (Simulated)
      display.setFont(&FreeSansBold14pt7b);
      display.setCursor(98, 20);
      display.print("24"); // Hardcoded for demo stability

      // Status Icon
      display.setFont(NULL);
      display.setCursor(118, 55);
      display.print("D"); // D for Demo
      break;
    }
    case 2: { // TIMERS
      display.setFont(NULL); display.setCursor(0, 0); display.print("Global Timer:");
      display.setFont(&FreeSansBold14pt7b); display.setCursor(0, 28);
      display.print(timerTransform(millis()));
      
      display.setFont(NULL); display.setCursor(0, 34); display.print("Drive Timer:");
      display.setFont(&FreeSansBold14pt7b); display.setCursor(0, 62);
      display.print(timerTransform(millis()/2)); // Fake drive time
      break;
    }
    case 3: { // TRIP DATA
      display.setFont(&FreeSansBold14pt7b);
      display.setCursor(0, 20);
      display.print(global_totalDistance, 2); 
      display.setFont(NULL); display.setCursor(display.getCursorX() + 3, 10); display.println("mi");

      display.setFont(&FreeSansBold14pt7b);
      display.setCursor(0, 43);
      display.print(global_totalFuel, 3); 
      display.setFont(NULL); display.setCursor(display.getCursorX() + 3, 33); display.println("gal");
      
      display.setCursor(0, 56);
      display.print("MAF: "); display.print(global_maf, 1); display.print(" g/s");
      break;
    }
    case 4: { // GAUGES
      display.setFont(NULL); display.setCursor(0, 5); display.print("RPM:");
      display.setFont(&FreeSansBold12pt7b); display.setCursor(45, 15);
      display.print((int)global_rpm);

      display.setFont(NULL); display.setCursor(0, 28); display.print("LOAD:");
      display.setFont(&FreeSansBold12pt7b); display.setCursor(45, 38);
      display.print((int)global_load); display.print("%");

      display.setFont(NULL); display.setCursor(0, 51); display.print("FUEL:");
      display.setFont(&FreeSansBold12pt7b); display.setCursor(45, 61);
      display.print((int)global_fuel); display.print("%");
      break;
    }
    case 5: { // 0-60 TIMER (Visual Demo)
      float current_mph = global_kph * 0.621371;
      
      display.setFont(&FreeSansBold14pt7b);
      display.setCursor(0, 45);
      display.print((int)current_mph);
      display.setFont(NULL);
      
      display.setCursor(65, 10); display.print("30: 2.5s"); // Fake data
      display.setCursor(65, 25); display.print("60: 5.8s");
      display.setCursor(65, 40); display.print("80: --.-");
      
      display.setCursor(0, 55); display.print("TESTING...");
      break;
    }
    case 6: { // DTC CODES
      display.setCursor(0, 0);
      display.println("DIAGNOSTIC TROUBLE\nCODES (DTC):");
      // Split the global simulated string
      char local_str[50];
      strcpy(local_str, global_dtcString);
      char *token = strtok(local_str, "\n");
      while (token != NULL) {
        display.println(token);
        token = strtok(NULL, "\n");
      }
      break;
    }
  }

  display.display();
  delay(10); // Small delay for stability
}