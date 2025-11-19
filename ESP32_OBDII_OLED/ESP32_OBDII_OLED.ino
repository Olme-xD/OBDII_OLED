/*
* ESP32_OBDII_OLED.ino
* Created by Olme-xD
* On November 15, 2025
* 
* WARNING:
  * Use ESP32 Board Version 2.0.14
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
volatile double global_totalDistanceTraveled = 0.0;
volatile double global_totalFuelConsumed = 0.0;
volatile uint32_t global_lastDataUpdate = 0;
volatile uint32_t global_lastPressed = millis();
volatile int global_mode = 1;
typedef enum {OBD_STATE_KPH, OBD_STATE_MAF, ODB_STATE_RPM, ODB_STATE_LOAD, ODB_STATE_FUEL} ObdState;

void obdTask(void *pvParameters) {
  DEBUG_PORT.println("OBD Task started on Core 0");

  static uint32_t lastSuccessfulMpgPollTime = 0;
  static ObdState currentState = OBD_STATE_KPH;
  static float_t temp_kph = 0.0;  // Store speed between states

  for (;;) {
    if (ELM_PORT.connected()) {
      switch (currentState) {
        case OBD_STATE_KPH: {
          float_t value = myELM327.kph();

          if (myELM327.nb_rx_state == ELM_SUCCESS) {
            temp_kph = value;

            // Update Global KPH
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
              global_vss_kph = temp_kph;
              global_lastDataUpdate = millis(); // Update timestamp here so the "*" stays alive even if MAF fails
              xSemaphoreGive(dataMutex);
            }

            currentState = OBD_STATE_MAF;
          } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
            myELM327.printError();
          }
          break;
        }
        case OBD_STATE_MAF: {
          float_t value = myELM327.mafRate();

          if (myELM327.nb_rx_state == ELM_SUCCESS) {
            float_t temp_maf = value;
            uint32_t pollCompleteTime = millis();
            double distance_delta_km = 0.0;
            double fuel_delta_l = 0.0;
            int local_mode = 1;

            // Fuel Rate (L/hr) = MAF * 0.3309
            float_t local_fuelRate_lph = temp_maf * 0.33094;

            if (lastSuccessfulMpgPollTime > 0) {
              double deltaTime_hours = (pollCompleteTime - lastSuccessfulMpgPollTime) / 3600000.0;
              fuel_delta_l = local_fuelRate_lph * deltaTime_hours;

              if (temp_kph > 0) {
                distance_delta_km = temp_kph * deltaTime_hours;
              }
            }
            lastSuccessfulMpgPollTime = pollCompleteTime;

            // Save & Retrive Global Variables
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
              global_maf = temp_maf;
              global_totalDistanceTraveled += distance_delta_km;
              global_totalFuelConsumed += fuel_delta_l;
              global_lastDataUpdate = millis();
              local_mode = global_mode;
              xSemaphoreGive(dataMutex);
            }

            if (local_mode == 3) {
              currentState = OBD_STATE_RPM;
            } else {
              currentState = OBD_STATE_KPH;
            }
          } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
            myELM327.printError();
            currentState = OBD_STATE_KPH;
          }
          break;
        }
        case ODB_STATE_RPM: {
          float_t value = myELM327.rpm();

          if(myELM327.nb_rx_state == ELM_SUCCESS) {

            // Update Global RPM
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
              global_rpm = value;
              xSemaphoreGive(dataMutex);
            }

            currentState = OBD_STATE_LOAD;
          } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
            myELM327.printError();
          }
          break;
        }
        case ODB_STATE_LOAD: {
          float_t value = myELM327.engineLoad();

          if(myELM327.nb_rx_state == ELM_SUCCESS) {

            // Update Global Engine Load
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
              global_load = value;
              xSemaphoreGive(dataMutex);
            }

            currentState = OBD_STATE_FUEL;
          } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
            myELM327.printError();
          }
          break;
        }
        case ODB_STATE_FUEL: {
          float_t value = myELM327.fuelLevel();

          if(myELM327.nb_rx_state == ELM_SUCCESS) {

            // Update Global Fuel Level
            if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
              global_fuel = value;
              xSemaphoreGive(dataMutex);
            }

            currentState = OBD_STATE_KPH;
          } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
            myELM327.printError();
          }
          break;
        }
        default: currentState = OBD_STATE_KPH;
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

void setup() {
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
  // Local Variables
  float_t local_kph, local_maf;
  double local_totalDist, local_totalFuel;
  uint32_t local_lastUpdate, local_lastPressed;
  int local_mode;
  
  // Read & Retrive Data to Global Variables
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    local_kph = global_vss_kph;
    local_maf = global_maf;
    local_totalDist = global_totalDistanceTraveled;
    local_totalFuel = global_totalFuelConsumed;
    local_lastUpdate = global_lastDataUpdate;
    global_lastPressed = local_lastPressed;
    global_mode = local_mode;
    xSemaphoreGive(dataMutex);
  } else {
    return;
  }

  // Switch Reading State
  if (digitalRead(SWITCH) == HIGH) {
    if (millis() > (local_lastPressed + 1000)) {
      local_lastPressed = millis();
      if (local_mode >= 5) {
        local_mode = 1;
      } else {
        local_mode++;
      }
    }
  }

  // Mode Switching
  switch(local_mode) {
    case 1: {
      // MPG & MPH calculations
      float_t local_mph = local_kph * 0.621371;
      float_t inst_mpg = 0.0;
      bool isInstantMpgValid = (local_kph > 0 && local_maf > 0);

      if (isInstantMpgValid) {
        inst_mpg = round((14.7 * 6.17 * 4.54 * local_kph * 0.621371) / (3600 * local_maf / 100));
        if (inst_mpg > 90.0) inst_mpg = 90.0;
      }

      float_t avg_mpg = 0.0;
      bool isAverageMpgValid = (local_totalDist > 0.1 && local_totalFuel > 0.001);

      if (isAverageMpgValid) {
        double avg_lp100k = (local_totalFuel / local_totalDist) * 100.0;
        avg_mpg = 235.21 / avg_lp100k;
      }

      // Clear display and show data
      display.clearDisplay();
      display.setTextColor(SSD1306_WHITE);

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
      if (millis() - local_lastUpdate < 2000) {
        display.print("*");
      } else if(!ELM_PORT.connected()) {
        display.print("!");
      } else {
        display.print("?");
      }

      display.display();
      delay(100);
      break;
    }
    case 2: {
      break;
    }
    case 3: {
      break;
    }
    case 4: {
      break;
    }
    case 5: {
      break;
    }
    default: local_mode = 1;
  }
}