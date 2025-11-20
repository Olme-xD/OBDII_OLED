/*
* ESP32_OBDII_OLED_DEMO.ino
* Created by Olme-xD
* Modified for DEMO MODE (Made with AI from Running Version)
*
* FEATURES:
  * SIMULATION MODE: Generates fake Speed, RPM, MAF, and Load data.
  * Dual-Core (FreeRTOS): Core 0 runs Physics Simulation, Core 1 updates display.
  * Calculates true average MPG based on simulated physics.
*/

// Libraries
#include <math.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "FreeSansBold40pt7b.h"
#include "FreeSansBold14pt7b.h"

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
#define DEBUG_PORT Serial
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TaskHandle_t simTaskHandle;
SemaphoreHandle_t dataMutex;

// Global Variables
volatile float_t global_vss_kph = 0.0;
volatile float_t global_maf = 0.0;
volatile float_t global_rpm = 0.0;
volatile float_t global_load = 0.0;
volatile float_t global_fuel = 100.0; // Start at 100%
volatile double global_totalDistanceTraveled = 0.0;
volatile uint32_t global_totalDistanceTraveledTimer = 0.0;
volatile double global_totalFuelConsumed = 0.0;
volatile uint32_t global_lastDataUpdate = 0;
volatile int global_mode = 1;

// Simulation Helper Variables
float sim_speed = 0.0;
float sim_rpm = 800.0;
int sim_phase = 0; // 0=Idle, 1=Accel, 2=Cruise, 3=Decel

void simulationTask(void *pvParameters) {
  /* * Function: simulationTask
   * Purpose: Runs on Core 0 to simulate a driving cycle.
   * Logic: 
   * - Generates synthetic data for Speed, RPM, and MAF.
   * - Performs the same physics integration (Dist = Speed*Time) as the real app.
   * - Cycles through Idle -> Accel -> Cruise -> Decel phases.
   */
  
  DEBUG_PORT.println("Simulation Task started on Core 0");

  uint32_t lastSimTime = millis();

  for (;;) {
    uint32_t now = millis();
    uint32_t time_delta_ms = now - lastSimTime;
    double time_hours = time_delta_ms / 3600000.0;
    lastSimTime = now;

    // --- 1. PHYSICS SIMULATION LOGIC ---
    // Change phases every few seconds for demo variety
    static uint32_t phaseTimer = 0;
    if (now - phaseTimer > 5000) { 
        sim_phase++;
        if (sim_phase > 3) sim_phase = 1; // Loop Accel->Cruise->Decel (Skip long idle)
        phaseTimer = now;
    }

    switch(sim_phase) {
        case 0: // IDLE
            sim_speed = 0;
            sim_rpm = 800 + random(-20, 20);
            global_maf = 2.5 + (random(-10, 10) / 10.0);
            global_load = 15;
            break;
        case 1: // ACCELERATE
            sim_speed += 0.8; // Gain speed
            if(sim_speed > 120) sim_speed = 120;
            sim_rpm = 2000 + (sim_speed * 30);
            global_maf = 20.0 + (sim_speed * 0.5) + random(0, 5);
            global_load = 80;
            break;
        case 2: // CRUISE
            sim_speed = 100 + (sin(now / 1000.0) * 2); // Gentle wave
            sim_rpm = 2200 + random(-50, 50);
            global_maf = 15.0 + random(-2, 2); // Low MAF = Good MPG
            global_load = 30;
            break;
        case 3: // DECELERATE
            sim_speed -= 1.5;
            if(sim_speed < 0) sim_speed = 0;
            sim_rpm = 1000 + (sim_speed * 10);
            global_maf = 3.0; // Coasting
            global_load = 10;
            break;
    }

    // --- 2. DATA INTEGRATION (Real Math) ---
    double dist_delta = 0.0;
    double fuel_delta = 0.0;

    // Distance Integration
    if (sim_speed > 0) {
        dist_delta = sim_speed * time_hours;
    }

    // Fuel Integration (L/hr = MAF * 0.33094)
    float fuel_rate = global_maf * 0.33094;
    fuel_delta = fuel_rate * time_hours;
    
    // Drain the fake fuel tank
    if (global_fuel > 0) global_fuel -= (fuel_delta * 5); // *5 to show it moving faster

    // --- 3. UPDATE GLOBALS SAFELEY ---
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        global_vss_kph = sim_speed;
        global_rpm = sim_rpm;
        
        global_totalDistanceTraveled += dist_delta;
        global_totalFuelConsumed += fuel_delta;
        
        if (sim_speed > 0) {
            global_totalDistanceTraveledTimer += time_delta_ms;
        }
        
        global_lastDataUpdate = now; 
        xSemaphoreGive(dataMutex);
    }

    // Update rate: 20Hz (Smooth animation)
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

String timerTransform(uint32_t millisTime) {
  /* * Function: timerTransform
   * Purpose: Helper utility to format a duration into a readable String.
   */
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

void setup() {
  /* * Function: setup
   * Purpose: DEMO Initialization.
   * Logic: Starts OLED and Simulation Task immediately.
   */
  DEBUG_PORT.begin(115200);
  pinMode(SWITCH, INPUT_PULLDOWN);

  // Initialize OLED Display
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    DEBUG_PORT.println(F("SSD1306 init failed"));
    ESP.restart();
  }

  // Show Startup Message
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OBD-II Reader\n");
  display.println("DEMO MODE ACTIVE");
  display.println("---------------------");
  display.println("Simulating Data...");
  display.display();
  delay(2000);

  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
    ESP.restart();
  }

  // Start the Simulation Task instead of the OBD Task
  xTaskCreatePinnedToCore(simulationTask, "SimTask", 4096, NULL, 1, &simTaskHandle, 0);
}

void loop() {
  /* * Function: loop
   * Purpose: Main Display Loop (Core 1).
   * Logic: Identical to original, but reads generated variables.
   */

  // Local Variables
  float_t local_kph, local_maf;
  double local_totalDistanceTraveled, local_totalFuel;
  uint32_t local_lastUpdate, local_totalDistanceTraveledTimer;
  static uint32_t local_lastPressed = 0;
  static bool buttonActive = false;
  static int local_mode = 3;

  // Switch Reading State
  if (digitalRead(SWITCH) == HIGH) {
    if (!buttonActive) {
      buttonActive = true;
      local_lastPressed = millis();
    }
    if ((millis() - local_lastPressed) > 1000) {
      local_lastPressed = millis();
      if (local_mode >= 4) { // Reduced modes for Demo
        local_mode = 1;
      } else {
        local_mode++;
      }
    }
  } else {
    buttonActive = false;
  }
  
  // Retrieve Data
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    local_kph = global_vss_kph;
    local_maf = global_maf;
    local_totalDistanceTraveled = global_totalDistanceTraveled;
    local_totalDistanceTraveledTimer = global_totalDistanceTraveledTimer;
    local_totalFuel = global_totalFuelConsumed;
    local_lastUpdate = global_lastDataUpdate;
    global_mode = local_mode;
    xSemaphoreGive(dataMutex);
  } else {
    return;
  }

  // Clear Display
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Mode Switching
  switch(local_mode) {
    case 1: {
      // MPG & MPH calculations
      float_t inst_mpg = 0.0;
      bool isInstantMpgValid = (local_kph > 0 && local_maf > 0);

      if (isInstantMpgValid) {
        inst_mpg = round((14.7 * 6.17 * 4.54 * local_kph * 0.621371) / (3600 * local_maf / 100));
        if (inst_mpg > 99.0) inst_mpg = 99.0;
      }

      float_t avg_mpg = 0.0;
      bool isAverageMpgValid = (local_totalDistanceTraveled > 0.1 && local_totalFuel > 0.001);

      if (isAverageMpgValid) {
        double avg_lp100k = (local_totalFuel / local_totalDistanceTraveled) * 100.0;
        avg_mpg = 235.21 / avg_lp100k;
      }

      // Instant MPG
      display.setFont(&FreeSansBold40pt7b);
      display.setCursor(0, 60);
      if (isInstantMpgValid) {
        display.print(round(inst_mpg), 0);
      } else if (local_kph == 0 && local_maf > 0) {
        display.print("00"); 
      } else {
        display.print("--");
      }

      // Average MPG
      display.setFont(&FreeSansBold14pt7b);
      display.setCursor(98, 20);
      if (isAverageMpgValid) {
        display.print(round(avg_mpg), 0);
      } else {
        display.print("--");
      }

      // Status Sign (D for Demo)
      display.setFont(NULL);
      display.setTextSize(1);
      display.setCursor(118, 55);
      display.print("D");
      break;
    }
    case 2: {
      display.setFont(NULL);
      display.setTextSize(1);

      // Display Distance Traveled
      display.println("Distance Traveled:");
      display.print((local_totalDistanceTraveled * 0.621371), 1);
      display.println(" mi");
      display.println();

      // Display Time While Above 0 KPH
      display.println("Drive Timer:");
      display.println(timerTransform(local_totalDistanceTraveledTimer));
      display.println();

      // Display Global Time
      display.println("Global Timer:");
      display.println(timerTransform(millis()));
      break;
    }
    case 3: {
      display.setFont(NULL);
      display.setTextSize(1);

      // Show Total Distance
      display.setCursor(0, 10);
      display.print("Dist: ");
      display.print(local_totalDistanceTraveled * 0.621371, 2); 
      display.print(" mi");

      // Show Total Fuel
      display.setCursor(0, 23);
      display.print("Fuel: ");
      display.print(local_totalFuel * 0.264172, 3); 
      display.print(" gal");

      // Calculated MPG Average
      display.setCursor(0, 43);
      display.print("Calc Avg: ");
      if (local_totalDistanceTraveled > 0.1) {
        double lp100k = (local_totalFuel / local_totalDistanceTraveled) * 100.0;
        display.print(235.21 / lp100k, 1);
      } else {
        display.print("--");
      }

      // Show Raw Sensor Data
      display.setCursor(0, 58);
      display.print("MAF: "); display.print(local_maf, 1);
      display.print(" g/s");
      break;
    }
    case 4: {
      float_t local_rpm = 0.0;
      float_t local_fuelGauge = 0.0;
      float_t local_engineLoad = 0.0;
      
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        local_rpm = global_rpm;
        local_fuelGauge = global_fuel;
        local_engineLoad = global_load;
        xSemaphoreGive(dataMutex);
      }

      display.setFont(NULL);
      display.setTextSize(2);

      // Display RPM
      display.setCursor(0, 10);
      display.print("RPM: ");
      display.println(local_rpm, 0);

      // Display Engine Load
      display.setCursor(0, 28);
      display.print("LOAD: ");
      display.print(local_engineLoad, 0);
      display.println("%");

      // Display Fuel Level
      display.setCursor(0, 43);
      display.print("FUEL: ");
      display.print(local_fuelGauge, 1);
      display.println("%");
      break;
    }
  }

  display.display();
  delay(50); // Faster refresh for smooth demo animation
}