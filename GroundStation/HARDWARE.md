# 🛠️ ESP32 + E22-T30D Ground Station Hardware Setup Guide

This document provides a detailed bill of materials (BOM), schematics, wiring pinouts, power calculation, and assembly instructions for building the portable, independent 18650 battery-powered ground station.

---

## 1. Bill of Materials (BOM)

| Component | Description | Est. Cost (TWD) | Note |
| :--- | :--- | :--- | :--- |
| **ESP32 NodeMCU-32S** | Standard 38-pin DevKit (ESP-WROOM-32 chip inside) | ~$100 | Dual-core, built-in USB-TTL (CP2102) |
| **E22-400T30D** | Ebyte SX1268 433MHz 30dBm (1W) UART LoRa Module | ~$250 | Or **E22-900T30D** (915MHz) depending on rocket |
| **433MHz SMA Antenna** | High-gain rubber ducky antenna or spring antenna | ~$50 - $150 | Must match module frequency |
| **18650 Battery Shield V3** | Integrated charge/discharge protection & 5V boost | ~$80 | Typically uses TP4056 + IP5306 or equivalent |
| **18650 Li-ion Battery** | Nominal 3.7V, capacity > 2500mAh (e.g., Samsung/LG) | ~$120 | Reclaim from old laptops or buy new |
| **Jumper Wires & Switch** | Female-to-Female dupont wires, slide switch | ~$30 | For breadboard or soldering |

---

## 2. Power Supply Design & Safety

### ⚠️ Critical Warning
> [!CAUTION]
> **NO DIRECT BATTERY TO ESP32 3.3V:**
> A fully charged 18650 battery outputs **4.2V**. The ESP32 chip's absolute maximum rating is **3.6V**. Connecting the 18650 directly to the 3.3V pin will burn out the ESP32.
>
> **NO DIRECT BATTERY TO ESP32 5V (VIN):**
> If you connect the battery (3.0V - 4.2V) to the ESP32 board's `5V/VIN` pin, it passes through an onboard LDO regulator (e.g. AMS1117-3.3) which has a **1.1V - 1.3V dropout voltage**. When the battery drops to its nominal **3.7V**, the output of the LDO becomes `3.7V - 1.1V = 2.6V`. This voltage is too low, causing the ESP32 to brown out, crash, or fail to start.

### The Solution: 18650 Battery Shield V3
By using an off-the-shelf **18650 Battery Shield V3**, we solve both issues:
1. **Charging Circuit:** Features a TP4056 lithium battery charger chip to safely charge the 18650 cell via Micro-USB/Type-C.
2. **Protection Circuit:** Features a DW01 (overcharge/over-discharge protection) to prevent battery swelling and degradation.
3. **5V Boost Converter:** Utilizes a highly efficient DC-DC boost converter (e.g. IP5306 or similar) to step up the battery voltage to a stable **5.0V** (capable of outputting up to 2.0A).
4. **Stable Power Distribution:**
   - The boosted stable **5V** is fed directly into the **E22 VCC** (ensures full 30dBm / 1W transmit power).
   - The boosted stable **5V** is fed into the **ESP32 5V/VIN** pin (the onboard 3.3V LDO now receives a steady 5V and outputs a clean 3.3V for the ESP32 chip).

---

## 3. Power Consumption & Battery Life Estimation

*   **ESP32 Power Consumption:**
    *   CPU Active (Receive only, no Wi-Fi/BT): **~60mA**
    *   Wi-Fi/Bluetooth enabled (optional): **~160mA - 240mA**
*   **E22-T30D Power Consumption:**
    *   Receive Mode (Listening): **~16mA**
    *   Transmit Mode (30dBm / 1W peak): **~650mA**

### Operational Lifespan Calculation
Assuming the ground station is primarily in **Receive Mode** (only listening to the rocket telemetry) and Wi-Fi is disabled:
$$\text{Total System Current} = 60\text{ mA (ESP32)} + 16\text{ mA (E22)} \approx 76\text{ mA}$$

Accounting for a $85\%$ boost converter efficiency:
$$\text{Average Draw from 3.7V Battery} = \frac{76\text{ mA} \times 5.0\text{V}}{3.7\text{V} \times 0.85} \approx 120\text{ mA}$$

For a standard **3000mAh** 18650 battery:
$$\text{Battery Life} = \frac{3000\text{ mAh}}{120\text{ mA}} \approx 25\text{ Hours}$$
The ground station can run continuously for **over 24 hours** on a single charge!

---

## 4. Hardware Wiring & Pin Mapping

Ensure all power is disconnected before wiring. Connect the pins as shown in the table below:

| E22-T30D Pin | Wire Color (Rec.) | ESP32 GPIO | Description | Note |
| :--- | :--- | :--- | :--- | :--- |
| **VCC** | Red | **5V / VIN** | 5V Power Input | Connect to 5V output solder pad of the Battery Shield |
| **GND** | Black | **GND** | Common Ground | Connect to GND solder pad of the Battery Shield |
| **TXD** | Yellow | **GPIO 16 (RX2)**| UART Data Out | Outputs received data from E22 to ESP32 RX2 |
| **RXD** | White | **GPIO 17 (TX2)**| UART Data In | Receives commands/telemetry from ESP32 TX2 |
| **AUX** | Blue | **GPIO 15** | State Indication | High = Idle, Low = Processing/Busy |
| **RST** | Purple | **GPIO 13** | Module Reset | Pull low to reset E22 hardware |
| **M0** | Orange | **GPIO 21** | Mode Control 0 | Controls working mode of E22 |
| **M1** | Brown | **GPIO 22** | Mode Control 1 | Controls working mode of E22 |

> [!IMPORTANT]
> **Common Ground:** Ensure the ESP32 Ground, E22 Ground, and the 18650 battery shield Ground are all tied together to establish a common reference voltage.

---

## 5. Wiring Schematic (Breadboard View)

```
       +---------------------------------------------+
       |           18650 Battery Shield              |
       |  [Bat Charger] ----> [5V Boost Converter]   |
       |                        |            |       |
       +------------------------|------------|-------+
                                | 5V         | GND
                                |            |
         +----------------------+            +--------+
         |                      |                     |
         v 5V/VIN               v VCC                 v GND
  +--------------+       +--------------+      +--------------+
  |              |       |              |      |              |
  |    ESP32     |       |   E22 T30D   |      | Common GND   |
  |  NodeMCU-32S |       |  LoRa Module |      |   Bus Bar    |
  |              |       |              |      |              |
  |    GPIO16    |<------|     TXD      |      |              |
  |    GPIO17    |------>|     RXD      |      |              |
  |    GPIO15    |<------|     AUX      |      |              |
  |    GPIO13    |------>|     RST      |      |              |
  |    GPIO21    |------>|     M0       |      |              |
  |    GPIO22    |------>|     M1       |      |              |
  |              |       |              |      |              |
  |     GND      |       |     GND      |      |              |
  +-------+------+       +-------+------+      +-------+------+
          |                      |                     |
          +----------------------o---------------------+
```

---

## 6. Step-by-Step Assembly Instructions

### Step 1: Install Antenna
*   **WARNING:** Always screw the SMA antenna onto the E22 module **BEFORE** applying power. Operating a transmitter without an antenna can instantly destroy the RF power amplifier due to impedance mismatch (high VSWR).

### Step 2: Assemble Power Source
*   Insert the 18650 battery cell into the slot on the battery shield. Verify the polarity matches the markings (`+` and `-`).
*   Charge the battery fully using a USB charger connected to the shield's charging port until the status LED changes to green/blue.

### Step 3: Connect Control and Logic Pins
*   Using female-to-female jumper wires (or soldering onto a perfboard), connect the E22 control pins (`M0`, `M1`, `AUX`, `RST`, `RXD`, `TXD`) to the designated ESP32 GPIOs as specified in Section 4.

### Step 4: Hook Up Power Rails
*   Connect the E22 `VCC` pin and the ESP32 `5V / VIN` pin to the `5V` output terminals/solder pads on the battery shield.
*   Connect both `GND` pins to the `GND` terminal/solder pad on the battery shield.

### Step 5: Double Check
*   Ensure there are no short circuits between `5V` and `GND`.
*   Ensure `TXD` goes to `GPIO16 (RX)` and `RXD` goes to `GPIO17 (TX)` (crossing the lines is required).

### Step 6: Power Up!
*   Slide the battery shield power switch to the `ON` position.
*   The ESP32 onboard LED will flash to indicate it has started up.
