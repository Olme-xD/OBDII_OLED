/*
* ESP32_OBDII_OLED.ino
* Created by Olme-xD
* On November 15, 2025
* 
* WARNING:
  * Use ESP32 Board Version 2.0.14
  * Use ELMDuino Library Version 3.3.0
*
* FEATURES:
  * Dual-Core (FreeRTOS): Core 0 polls OBD (State Machine), Core 1 updates display.
  * Connects via Bluetooth MAC address (ELM327) ONLY.
  * Uses MAF & KPH sensor for MPG calculations.
  * Calculates true average MPG.
  * Uses State Machine for stable data retrieval.
*/

// Libraries
#include <math.h>
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

// Switch Pin Set
#define SWITCH 12

// OBDII Dongle Mac Address
uint8_t address[6] = {0xaa, 0xbb, 0xcc, 0x11, 0x22, 0x33};

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
volatile float_t global_rpm = -1.0;
volatile float_t global_load = -1.0;
volatile float_t global_fuel = -1.0;
#define FUEL_CORRECTION 1.10 // Calibration for Fuel Readings
volatile double global_totalDistanceTraveled = 0.0;
volatile uint32_t global_totalDistanceTraveledTimer = 0.0;
volatile double global_totalFuelConsumed = 0.0;
volatile uint32_t global_lastDataUpdate = 0;
volatile int global_mode = 1;
typedef enum {OBD_STATE_KPH, OBD_STATE_MAF, OBD_STATE_RPM, OBD_STATE_LOAD, OBD_STATE_FUEL, OBD_STATE_DTC} ObdState;

void obdTask(void *pvParameters) {
  /* * Function: obdTask
  * Purpose: Runs on Core 0 via FreeRTOS to handle all Bluetooth communication.
  * Logic: 
  * - Implements a State Machine to cycle through polling specific PIDs (Speed, MAF, RPM, etc.).
  * - Performs physics calculations (Distance = Speed * Time, Fuel = MAF * Time) between polls.
  * - Updates global variables safely using a Mutex (semaphore) to prevent data race conditions with the display loop.
  * - Handles automatic reconnection if the Bluetooth link drops.
  */

  DEBUG_PORT.println("OBD Task started on Core 0");

  static uint32_t lastSuccessfulMpgPollTime = 0;
  static ObdState currentState = OBD_STATE_KPH;
  static float_t temp_kph = 0.0;
  static float_t prev_kph = 0.0;
  static float_t prev_maf = 0.0;
  static uint32_t lastSpeedTime = 0;
  static uint32_t lastFuelTime = 0;

  for (;;) {
    if (ELM_PORT.connected()) {
      switch (currentState) {
        case OBD_STATE_KPH: {
          float_t value = myELM327.kph();
          uint32_t now = millis();

          if (myELM327.nb_rx_state == ELM_SUCCESS) {
            temp_kph = value;
            double dist_delta = 0.0;
            uint32_t time_delta_ms = 0;

            // Calculate if NOT the First Loop
            if (lastSpeedTime > 0) {
              time_delta_ms = now - lastSpeedTime;
              double time_hours = time_delta_ms / 3600000.0;

              // Distance = Average of (Current + Prev) * Time
              if (temp_kph > 0 || prev_kph > 0) {
                float avg_speed = (temp_kph + prev_kph) / 2.0;
                dist_delta = avg_speed * time_hours;
              }
            }

            // Update Variables after Calculations
            lastSpeedTime = now;
            prev_kph = temp_kph;

            // Save to Globals
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
              global_vss_kph = temp_kph;
              global_totalDistanceTraveled += dist_delta;
              if (temp_kph > 0) global_totalDistanceTraveledTimer += time_delta_ms;
              global_lastDataUpdate = now; 
              xSemaphoreGive(dataMutex);
            }

            currentState = OBD_STATE_MAF;
          } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
            myELM327.printError();
          }
          break;
        }
        case OBD_STATE_MAF: {
          float_t value = myELM327.mafRate() * FUEL_CORRECTION;
          uint32_t now = millis();

          if (myELM327.nb_rx_state == ELM_SUCCESS) {
            float_t temp_maf = value;
            double fuel_delta = 0.0;
            int local_mode = 1;

            // Calculate if NOT the First Loop
            if (lastFuelTime > 0) {
              double time_hours = (now - lastFuelTime) / 3600000.0;

              // Fuel Rate (L/hr) = MAF * 0.33094 && Average of (Current + Prev) * Time
              if (temp_maf > 0 || prev_maf > 0) {
                float avg_maf = (temp_maf + prev_maf) / 2.0;
                float avg_fuel_rate = avg_maf * 0.33094;
                fuel_delta = avg_fuel_rate * time_hours;
              }
            }

            // Update Variables after Calculations
            lastFuelTime = now;
            prev_maf = temp_maf;
            
            // Save to Global (Mutex Block)
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
              global_maf = temp_maf;
              global_totalFuelConsumed += fuel_delta;
              global_lastDataUpdate = now;
              local_mode = global_mode;
              xSemaphoreGive(dataMutex);
            }

            // Selective Polling Logic
            if (local_mode == 4) {
              currentState = OBD_STATE_RPM;
            } else if (local_mode == 6) {
              currentState = OBD_STATE_DTC;
            } else {
              currentState = OBD_STATE_KPH;
            }
          } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
            myELM327.printError();
            currentState = OBD_STATE_KPH;
          }
          break;
        }
        case OBD_STATE_RPM: {
          float_t value = myELM327.rpm();

          if(myELM327.nb_rx_state == ELM_SUCCESS) {

            // Update Global RPM
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
              global_rpm = value;
              global_lastDataUpdate = millis();
              xSemaphoreGive(dataMutex);
            }

            currentState = OBD_STATE_LOAD;
          } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
            myELM327.printError();
            currentState = OBD_STATE_LOAD;
          }
          break;
        }
        case OBD_STATE_LOAD: {
          float_t value = myELM327.engineLoad();

          if(myELM327.nb_rx_state == ELM_SUCCESS) {

            // Update Global Engine Load
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
              global_load = value;
              global_lastDataUpdate = millis();
              xSemaphoreGive(dataMutex);
            }

            currentState = OBD_STATE_FUEL;
          } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
            myELM327.printError();
            currentState = OBD_STATE_FUEL;
          }
          break;
        }
        case OBD_STATE_FUEL: {
          float_t value = myELM327.fuelLevel();

          if(myELM327.nb_rx_state == ELM_SUCCESS) {

            // Update Global Fuel Level
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
              global_fuel = value;
              global_lastDataUpdate = millis();
              xSemaphoreGive(dataMutex);
            }

            currentState = OBD_STATE_KPH;
          } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
            myELM327.printError();
            currentState = OBD_STATE_KPH;
          }
          break;
        }
        case OBD_STATE_DTC: {
          break;
        }
      }
    } else {
      // Handle Reconnection
      DEBUG_PORT.println("OBD Task: Connection lost, trying to reconnect...");
      if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
        global_vss_kph = -1.0;
        xSemaphoreGive(dataMutex);
      }

      if (!ELM_PORT.connect(address)) {
        DEBUG_PORT.println("MAC Reconnection Failed.");
      }

      if (ELM_PORT.connected()) {
        DEBUG_PORT.println("Reconnected! Initializing ELM...");
        myELM327.begin(ELM_PORT, true, 2000);
      }

      lastSuccessfulMpgPollTime = 0;
      vTaskDelay(pdMS_TO_TICKS(2000));
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

String timerTransform(uint32_t millisTime) {
  /* * Function: timerTransform
  * Purpose: Helper utility to format a duration into a readable String.
  * Logic: 
  * - Takes a time input in milliseconds.
  * - math to break it down into Hours, Minutes, and Seconds.
  * - Returns a formatted String (e.g., "01:05:30" or "05:30") for the OLED display.
  */

  String finalTimer = "";

  // Calculate hours, minutes, and seconds
  uint16_t seconds = millisTime / 1000;
  uint16_t hours = seconds / 3600;
  seconds %= 3600;
  uint16_t minutes = seconds / 60;
  seconds %= 60;

  // Format Hours
  if (hours >= 1) {
    if (hours < 10) {
        finalTimer += '0';
    }
    finalTimer += String(hours) + ":";
  }

  // Format Minutes
  if (minutes < 10) {
      finalTimer += '0';
  }
  finalTimer += String(minutes) + ":";

  // Format Seconds
  if (seconds < 10) {
      finalTimer += '0';
  }
  finalTimer += String(seconds);

  return finalTimer;
}

void setup() {
  /* * Function: setup
  * Purpose: Standard Arduino initialization routine.
  * Logic: 
  * - Initializes Serial (Debug) and the OLED display.
  * - Connects to the ELM327 OBDII dongle via the specific MAC address.
  * - Creates the Mutex for thread-safe data sharing.
  * - Pins 'obdTask' to Core 0 to run in the background parallel to the main loop.
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
  display.println("Dual Core (FreeRTOS)");
  display.println("---------------------");
  display.println("Connecting via MAC...");
  display.display();

  // Connect to ELM327
  ELM_PORT.begin("OBDII OLED", true);

  // Try to connect via MAC ONLY
  if (!ELM_PORT.connect(address)) {
    DEBUG_PORT.println("Connection Failed (MAC)");
    display.println("\nConnection Failed!");
    display.display();
    delay(2000);
    ESP.restart();
  }

  if (!myELM327.begin(ELM_PORT, true, 2000)) {
    DEBUG_PORT.println("ELM Init Failed");
    display.println("\nELM Init Failed!");
    display.display();
    ESP.restart();
  }

  DEBUG_PORT.println("CONNECTED!");
  display.println("\nCONNECTED!");
  display.display();
  delay(500);

  dataMutex = xSemaphoreCreateMutex();
  if (dataMutex == NULL) {
    ESP.restart();
  }

  xTaskCreatePinnedToCore(obdTask, "OBDTask", 8192, NULL, 1, &obdTaskHandle, 0);
}

void loop() {
  /* * Function: loop
  * Purpose: The main User Interface task running on Core 1.
  * Logic: 
  * - Reads the physical button state to switch between display modes (1-6).
  * - Retrieves the latest sensor data from Global variables (using Mutex to ensure data integrity).
  * - Performs display-specific math (Instant MPG, L/100km, Average MPG).
  * - Draws the specific text and numbers to the OLED screen based on the selected 'local_mode'.
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
      local_lastPressed = millis(); // Reset Timer
    }
    if ((millis() - local_lastPressed) > 1000) {
      local_lastPressed = millis();
      if (local_mode >= 6) {
        local_mode = 1;
      } else {
        local_mode++;
      }
    }
  } else {
    buttonActive = false;
  }
  
  // Read & Retrive Data to Global Variables
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
      float_t local_mph = local_kph * 0.621371;
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
        display.print("00"); // If Speed is 0 but MAF is valid, we are idling. Show "0".
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

      // Status Sign
      display.setFont(NULL);
      display.setTextSize(1);
      display.setCursor(118, 55);
      if (millis() - local_lastUpdate < 200) {
        display.print("o");
      } else if(!ELM_PORT.connected()) {
        display.print("x");
      } else {
        display.print("!");
      }
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
      
      // Read & Retrive Data to Global Variables
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        local_rpm = global_rpm;
        local_fuelGauge = global_fuel;
        local_engineLoad = global_load;
        xSemaphoreGive(dataMutex);
      } else {
        return;
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
      display.print(local_fuelGauge, 2);
      display.println("%");
      break;
    }
    case 5: {
      static uint32_t startTime = 0;
      static bool isRunning = false;
      static float timer_30 = 0.0; 
      static float timer_60 = 0.0;
      static float timer_80 = 0.0;
      float current_mph = local_kph * 0.621371;

      // Reset State
      if (local_kph == 0) {
        isRunning = false;
        timer_30 = 0.0;
        timer_60 = 0.0;
        timer_80 = 0.0;
      }

      // Start Trigger
      if (current_mph > 0.5 && !isRunning && timer_80 == 0.0) {
        startTime = millis();
        isRunning = true;
      }

      // Running & Latching Logic
      if (isRunning) {
        float currentDuration = (millis() - startTime) / 1000.0;

        // Latch 30 MPH
        if (current_mph >= 30.0 && timer_30 == 0.0) {
          timer_30 = currentDuration;
        }

        // Latch 60 MPH
        if (current_mph >= 60.0 && timer_60 == 0.0) {
          timer_60 = currentDuration;
        }

        // Latch 80 MPH (Final Stop)
        if (current_mph >= 80.0 && timer_80 == 0.0) {
          timer_80 = currentDuration;
          isRunning = false; // Stop the main timer
        }

        // Resets Timer After 3 Minutes Regarless of Any State
        if (currentDuration > 180.0) isRunning = false;
      }

      // Speed
      display.setTextSize(3);
      display.setCursor(0, 25);
      display.print((int)current_mph);
      display.setTextSize(1);
      
      // 0-30 Line
      display.setCursor(65, 10);
      display.print("30: ");
      if (timer_30 > 0) display.print(timer_30, 2);
      else display.print(" --.-");

      // 0-60 Line
      display.setCursor(65, 25);
      display.print("60: ");
      if (timer_60 > 0) display.print(timer_60, 2);
      else display.print(" --.-");

      // 0-80 Line
      display.setCursor(65, 40);
      display.print("80: ");
      if (timer_80 > 0) display.print(timer_80, 2);
      else display.print(" --.-");

      // Status
      display.setCursor(0, 55);
      if (local_kph == 0) display.print("READY!");
      break;
    }
    case 6: {
      // DTC CODE DISPLAY
      break;
    }
  }

  // Display for Every Mode
  display.display();
  delay(100);
}