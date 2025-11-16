/*
* ESP32_OBDII_OLED.ino
* Created by Olme-xD
* On November 15, 2025
*
* MODIFIED FOR DUMMY DATA / SIMULATION
* (Original logic is preserved)
*/

// Libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ELMduino.h>
#include "BluetoothSerial.h"
#include "FreeSansBold40pt7b.h"
#include "FreeSansBold14pt7b.h"

// OLED Screen Settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define I2C_SDA 21
#define I2C_SCL 22

// OBDII Dongle Mac Address
uint8_t address[6] = { 0xaa, 0xbb, 0xcc, 0x11, 0x22, 0x33 };

// Instances
BluetoothSerial SerialBT;
#define ELM_PORT SerialBT
#define DEBUG_PORT Serial
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
ELM327 myELM327;
TaskHandle_t obdTaskHandle;
SemaphoreHandle_t dataMutex;

// Global Variables
volatile float_t global_vss_kph = -1.0;
volatile float_t global_maf = -1.0;
volatile double global_totalDistanceTraveled = 0.0;
volatile double global_totalFuelConsumed = 0.0;
volatile uint32_t global_lastDataUpdate = 0;

void obdTask(void *pvParameters) {
  DEBUG_PORT.println("OBD Task started on Core 0");

  static uint32_t lastSuccessfulMpgPollTime = 0;

  // Variable to cycle through dummy data states
  static int dummyState = 0;

  for (;;) {
    // if (ELM_PORT.connected()) { // Replaced with if(true) for simulation
    if (true) {

      // --- DUMMY DATA GENERATION ---
      // This block replaces the real myELM327 calls
      float_t local_kph;
      float_t local_maf;

      switch (dummyState) {
        case 0:              // State 0: Cruising (Good MPG)
          local_kph = 90.0;  // ~56 mph
          local_maf = 20.0;  // Moderate MAF
          break;
        case 1:  // State 1: Accelerating (Bad MPG)
          local_kph = 95.0;
          local_maf = 45.0;  // High MAF
          break;
        case 2:  // State 2: Idle (Should show "Idle")
          local_kph = 0.0;
          local_maf = 2.5;  // Low idle MAF
          break;
        case 3:  // State 3: Timeout (Should show "---")
          local_kph = -1.0;
          local_maf = -1.0;
          break;
      }
      // Cycle to the next state
      dummyState = (dummyState + 1) % 4;
      // --- END DUMMY DATA ---


      // Poll Primary PIDs
      // float_t local_kph = myELM327.kph();   // REPLACED
      // float_t local_maf = myELM327.mafRate(); // REPLACED
      uint32_t pollCompleteTime = millis();

      //
      // --- ALL YOUR ORIGINAL LOGIC IS UNCHANGED FROM HERE ---
      //

      // Check Data Validity
      bool isVssValid = (local_kph > -1);
      bool isMafValid = (local_maf > -1);
      bool isAnyDataValid = (isVssValid || isMafValid);
      bool isMpgDataValid = (isVssValid && isMafValid && local_kph > 0);

      double distance_delta_km = 0.0;
      double fuel_delta = 0.0;

      // The code for FUEL was provided by AI, not sure if it actually works.
      // Calculate Deltas (if valid)
      if (isMpgDataValid) {
        // Fuel Rate (L/hr) = (local_maf * 3600) / (14.7 * 740) = local_maf * 0.3309
        float_t local_fuelRate_lph = local_maf * 0.33094;

        if (lastSuccessfulMpgPollTime > 0) {
          double deltaTime_hours = (pollCompleteTime - lastSuccessfulMpgPollTime) / 3600000.0;
          distance_delta_km = local_kph * deltaTime_hours;
          fuel_delta = local_fuelRate_lph * deltaTime_hours;
        }
        lastSuccessfulMpgPollTime = pollCompleteTime;
      }

      // Update Global Variables (Critical Section)
      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        global_vss_kph = isVssValid ? local_kph : -1;
        global_maf = isMafValid ? local_maf : -1;

        if (isAnyDataValid) {
          global_lastDataUpdate = pollCompleteTime;
        }

        global_totalDistanceTraveled += distance_delta_km;
        global_totalFuelConsumed += fuel_delta;

        xSemaphoreGive(dataMutex);
      }

      // Add a delay to simulate polling rate
      vTaskDelay(pdMS_TO_TICKS(2000));  // Change state every 2 seconds
    }
    /* // Reconnection block removed for simulation
   else {
     // Handle Reconnection
     DEBUG_PORT.println("OBD Task: Connection lost, trying to reconnect...");
     if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
       global_vss_kph = global_maf = -1.0;
       xSemaphoreGive(dataMutex);
     }
     
     myELM327.begin(ELM_PORT, true, 2000);
     lastSuccessfulMpgPollTime = 0; 
     vTaskDelay(pdMS_TO_TICKS(3000)); // Wait 3s before retry
   }
   */
  }
}

void setup() {
  DEBUG_PORT.begin(115200);

  // Initialize OLED Display
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    DEBUG_PORT.println(F("SSD1306 init failed"));
    ESP.restart();
  }

  // Show Startup Message on OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("ESP32 OBD-II Reader");
  display.println("Dual Core (MAC)");
  display.println("-----------------");
  // display.println("Connecting via MAC..."); // Changed for simulation
  display.println("SIMULATION MODE");
  display.display();

  // Connect to ELM327 via MAC Address
  // ELM_PORT.begin("ESP32_OBD_OLED", true); // SKIPPED

  /* --- SIMULATION: Auto-connect ---
 if (!ELM_PORT.connect(address)) {
   DEBUG_PORT.println("CONNECTION ERROR ->> PHASE #1");
   display.println("\nBT Connect Failed!");
   display.display();
   delay(2000);
   ESP.restart();
 }
 
 if (!myELM327.begin(ELM_PORT, true, 2000)) {
   DEBUG_PORT.println("CONNECTION ERROR ->> PHASE #2");
   display.println("\nELM Init Failed!");
   display.display();
   delay(2000);
   ESP.restart();
 }
 */

  DEBUG_PORT.println("CONNECTED TO ELM327! (SIMULATED)");
  display.println("\nCONNECTED!");
  display.display();
  delay(200);

  // Create the Mutex and Core 0 Task
  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
    DEBUG_PORT.println("Mutex creation failed!");
    ESP.restart();
  }

  xTaskCreatePinnedToCore(obdTask, "OBDTask", 8192, NULL, 1, &obdTaskHandle, 0);
}

void loop() {
  //
  // --- YOUR ORIGINAL loop() IS 100% UNCHANGED ---
  //

  // Read Shared Data (Atomic)
  float_t local_kph, local_maf;
  double local_totalDist, local_totalFuel;
  uint32_t local_lastUpdate;

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    local_kph = global_vss_kph;
    local_maf = global_maf;
    local_totalDist = global_totalDistanceTraveled;
    local_totalFuel = global_totalFuelConsumed;
    local_lastUpdate = global_lastDataUpdate;
    xSemaphoreGive(dataMutex);
  } else {
    return;  // Skip this frame
  }

  // Perform Display Calculations
  float_t local_mph = local_kph * 0.621371;
  float_t inst_mpg = 0.0;
  bool isInstantMpgValid = (local_kph > 0 && local_maf > 0);

  if (isInstantMpgValid) {
    inst_mpg = (14.7 * 6.17 * 4.54 * local_kph * 0.621371) / (3600 * local_maf / 100);
    if (inst_mpg > 90) inst_mpg = 90;
  }

  // The code for AVERAGE MPG was provided by AI, not sure if it actually works.
  // Average MPG Calculation (True Avg) ... TRULY ACCURATE IF FUEL IS ACCURATE
  float_t avg_mpg = 0.0;
  bool isAverageMpgValid = (local_totalDist > 0 && local_totalFuel > 0);

  if (isAverageMpgValid) {
    double avg_lp100k = (local_totalFuel / local_totalDist) * 100.0;
    avg_mpg = 235.21 / avg_lp100k;
  }

  // Update the OLED Display
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Instant MPG (Large)
  display.setFont(&FreeSansBold40pt7b);
  // 'y=63' places the font baseline at the bottom of the 64px screen
  display.setCursor(0, 60);

  if (isInstantMpgValid) {
    display.print(round(inst_mpg), 0);
  } else if (local_kph == 0) {
    display.print("--");
  } else {
    display.print("--");
  }

  // Average MPG
  // Set to top-right corner
  display.setFont(&FreeSansBold14pt7b);
  // 'y=14' is a good top-margin for this font size
  display.setCursor(98, 20);
  if (isAverageMpgValid) {
    display.print(round(avg_mpg), 0);
  } else {
    display.print("--");
  }

  // Status Sign
  display.setFont(NULL);  // Use NULL to go back to the default font
  display.setTextSize(1);
  display.setCursor(118, 55);  // Bottom-right corner
  if (millis() - local_lastUpdate < 500) {
    display.print("*");  // "Live"
  } else {
    display.print("?");  // "Stale"
  }

  display.display();

  // Control screen refresh rate
  delay(100);  // ~10 times/sec
}