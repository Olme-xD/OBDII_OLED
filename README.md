# ESP32 OBD-II OLED Dashboard

A high-performance, Dual-Core OBD-II reader for ESP32. This project utilizes FreeRTOS to run heavy Bluetooth communication and physics calculations on **Core 0**, while maintaining a smooth, non-blocking OLED interface on **Core 1**.

## ⚠️ Critical Dependencies
**Strict adherence to these versions for stability:**

| Component | Version / Note |
| :--- | :--- |
| **ESP32 Board Manager** | **v2.0.14** (Do not use 3.0.x) |
| **ELMduino Library** | **v3.3.0** |

## 🔌 Pinout & Wiring

| Component | ESP32 Pin | Connection Type |
| :--- | :--- | :--- |
| **OLED SDA** | GPIO **21** | I2C Data |
| **OLED SCL** | GPIO **22** | I2C Clock |
| **Multi-Function Button** | GPIO **12** | Switch to **GND** (Active Low) |

*> **Note:** The code uses internal `INPUT_PULLUP`. Connect the switch between Pin 12 and Ground. No external resistor needed.*

## ⚙️ Calibration

### Fuel & MPG Correction
Adjust the fuel calculation scalar in the code to match your vehicle's actual consumption vs. sensor data.

* **Location:** Top of `ESP32_OBDII_OLED.ino`
* **Current Setting:** `1.08` (+8%)

### Bluetooth MAC Address
You must hardcode your ELM327's MAC address in the file.

## 🖥️ Interface & Modes

The device features a **Multi-Mode Interface**. Tap the button `X` times quickly, then wait 1 second to switch modes.

| # Mode | Mode Description | Features |
| :--- | :--- | :--- |
| **1** | **Efficiency** | Instant MPG (Big), Avg MPG, Connection Status |
| **2** | **Trip Data** | Global Timer, Drive Timer |
| **3** | **Stats** | Total Distance, Total Fuel Used, Calculated Avg MPG, MAF (g/s) |
| **4** | **Gauges** | RPM, Engine Load %, Fuel Level % |
| **5** | **Drag Timer** | 0-30, 0-60, 0-80 0-100 MPH (Auto-reset on stop) |
| **6** | **Diagnostics** | DTC Reader (Check Engine Codes) |

## 🧠 System Architecture

* **Core 0 (OBD Task):** Handles Bluetooth serial via `ELMduino`. Implements a State Machine to poll PIDs (Speed -> MAF -> RPM) and performs physics integration (Distance = Speed * Time) protected by Mutex.
* **Core 1 (Loop):** Handles OLED drawing, Multi-mode button logic, and overlays. Non-blocking design ensures gauges remain smooth during data retrieval.