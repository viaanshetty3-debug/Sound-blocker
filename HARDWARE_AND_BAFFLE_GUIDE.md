# Open-Window Active Noise Control (ANC) — Hardware Schematic & Window Baffle Guide

This guide covers:
1. **Complete Pin-by-Pin Circuit Schematic** (Teensy 4.0 + Analog Low-Pass Filters + MCP4921 SPI DAC + Power Amp)
2. **Analog Anti-Aliasing & Reconstruction Filter Values** (Zero-latency op-amp front-end)
3. **Mechanical Window Baffle Insert Design** (Hybrid passive high-frequency labyrinth + active bass-cancelling woofer core)

---

## Part 1: Electronic Circuit & Pinout Schematic

### Why This Exact Signal Chain?
To cancel random outdoor traffic rumble, your electronics must take **less than 1 millisecond** from microphone to speaker cone. Standard USB/I2S audio boards use Sigma-Delta converters that delay audio by $1\text{–}5\text{ ms}$. 
This circuit uses **SAR ADCs** and an **SPI String DAC (`MCP4921`)** with analog $500\text{ Hz}$ active low-pass filters, achieving an end-to-end electronic delay of **$< 10\text{ microseconds}$ ($0.01\text{ ms}$)**.

### Master Wiring Diagram

```
====================================================================================
1. MICROPHONE INPUT STAGE (Build 2 Identical Channels: Outdoor Ref & Indoor Error)
====================================================================================

 [Electret Mic] ---> [Preamp + 500 Hz Low-Pass Filter] ---> Teensy 4.0 ADC Pin
  (e.g. Primo         (MAX9814 Module w/ AGC OFF)           - Outdoor Ref -> Pin A0 (14)
   EM258 or            + Simple 2-Pole RC Filter:            - Indoor Err  -> Pin A1 (15)
   Cardioid            [R1 = 1.5 kΩ] --+-- [R2 = 1.5 kΩ] --+--> To Teensy A0 / A1
   Capsule)                            |                   |
                                  [C1 = 220 nF]       [C2 = 220 nF]
                                       |                   |
                                      GND                 GND
  * Cutoff Frequency: fc = 1 / (2 * pi * 1500 * 220e-9) = 482 Hz
  * Bias Voltage: MAX9814 outputs a 1.25V DC bias (or add two 10k resistors between
    3.3V and GND after a 4.7uF AC-coupling cap to center right at 1.65V).

====================================================================================
2. TEENSY 4.0 TO MCP4921 (12-BIT SPI DAC) & AMPLIFIER OUTPUT STAGE
====================================================================================

  Teensy 4.0                       MCP4921 (DIP-8)
  ----------                       ---------------
  3.3V      ---------------------> Pin 1 (VDD) & Pin 6 (VREF)
  Pin 10    ---------------------> Pin 2 (/CS)
  Pin 13    ---------------------> Pin 3 (SCK)
  Pin 11    ---------------------> Pin 4 (SDI / MOSI)
  GND       ---------------------> Pin 5 (/LDAC - tie directly to GND) & Pin 7 (VSS)
                                   Pin 8 (VOUT)
                                        |
                                   [C_dc = 10 uF Film/Electrolytic Cap] (Blocks 1.65V DC)
                                        |
                                   [R3 = 1.5 kΩ] ---+---> To Power Amp Analog IN
                                                    |
                                               [C3 = 220 nF] (Reconstruction Filter)
                                                    |
                                                   GND

====================================================================================
3. POWER AMPLIFIER & WOOFER
====================================================================================

  [Reconstruction Filter Out] ---> [10k Audio Potentiometer (Volume)]
                                        |
                                   [LM1875 / TDA2050 Class-AB Amp (20W-30W)]
                                        |
                                   [5.25" or 6.5" Woofer in Sealed Box (4Ω or 8Ω)]
```

### MAX9814 Microphone Preamp Wiring (x2 Modules)
If using off-the-shelf **MAX9814** breakout boards:
- **`VDD`** $\rightarrow$ Teensy `3.3V` (Use a $100\text{ }\Omega$ resistor + $47\text{ }\mu\text{F}$ cap to `GND` to filter power supply noise)
- **`GND`** $\rightarrow$ Teensy `GND`
- **`GAIN`** $\rightarrow$ Tie to **`VDD`** ($40\text{ dB}$ gain) or leave unconnected ($60\text{ dB}$ gain) depending on street loudness
- **`A/R`** $\rightarrow$ Leave unconnected, and **jumper the internal AGC cap or use fixed-gain OPA2134** if traffic volume swings wildly (ideally both mics should have fixed, linear gain so the adaptive filter doesn't fight an Automatic Gain Control circuit).

---

## Part 2: The Window Baffle Mechanical Design (Hybrid Active + Passive)

### Why You Need a Window Insert Box
If a window is wide open ($60\text{+}\text{ cm}$), high-frequency street noise ($500\text{–}4000\text{ Hz}$) and wind rush right past the speaker. By building a removable **Window Baffle Insert** (similar to a portable air conditioner window panel, made from $12\text{ mm}$ / $1/2\text{"}$ plywood or MDF), you combine **two technologies**:
1. **Passive Acoustic Labyrinth (Blocks $> 350\text{ Hz}$):** A foam-lined S-curve air channel lets fresh outdoor breeze flow into the room while absorbing high-frequency tire hiss, voices, and sirens.
2. **Active Bass Woofer Core (Cancels $40\text{–}300\text{ Hz}$):** Low-frequency bass travels effortlessly through the foam channel, where the **sealed $5.25\text{"}$ woofer** fires an inverted wave directly into the duct exit to cancel the rumble before it enters the room.

### Cross-Section Diagram of the Window Baffle Insert (Top-Down View)

```
                     OUTSIDE (Street Side)
       ==================================================
                  |                        |
       (Ref Mic)  |     35 cm Aluminum     |
         [🎤] <---+---  Mic Boom Arm       |
      (Windscreen)|                        |
                  v                        |
       +----------+------------------------+------------+  <-- Window Frame
       | [Weatherproof Louver / Bug Screen]             |
       |                                                |
       |   AIR INTAKE                                   |
       |      |       +------------------------+        |
       |      |       |///////// FOAM /////////|        |
       |      v       +------------------------+        |
       |  +-------------------+                         |
       |  |////// FOAM ///////|   S-Curve Lined         |  12mm MDF / Plywood
       |  +-------------------+   Ventilation Channel   |  Baffle Box
       |                      |   (Absorbs >350 Hz)     |  (25 cm wide)
       |                      v                         |
       |      +--------------------------------+        |
       |      |     SEALED WOOFER CHAMBER      |        |
       |      |         (3.5 Liters)           |        |
       |      |   (Fills w/ Polyfill Stuffing) |        |
       |      +---------------+----------------+        |
       |      |  AIR + BASS   |  🔊 5.25" Woofer        |
       |      |   EXIT SLOT   |  (Fires into exit zone) |
       +------+---------------+-------------------------+  <-- Window Frame
                      |               |
                      +-------+-------+
                              | 25 cm
                              v
                            [🎤] (Error Mic - Indoor Side)
                              
                     INSIDE (Room Side)
```

### Key Dimensions & Construction Rules

1. **Outer Box Width:** $22\text{–}25\text{ cm}$ ($9\text{–}10\text{ inches}$) wide, sized vertically to fit your sliding window track (use foam weatherstripping along the edges for an airtight perimeter seal).
2. **Air Channel Width:** $8\text{–}10\text{ cm}$ ($3\text{–}4\text{ inches}$) clear airway lined on both walls with **$25\text{ mm}$ ($1\text{"}$) acoustic open-cell melamine or polyurethane foam**. Two $90^\circ$ bends block direct line-of-sight from the street (stopping high-frequency noise) while allowing steady airflow.
3. **Sealed Woofer Sub-Enclosure:**
   - Internal volume: **$3.0\text{ to }4.5\text{ Liters}$** ($0.10\text{–}0.16\text{ ft}^3$), completely airtight and stuffed with loose polyester fiberfill (pillow stuffing).
   - Mount the **$5.25\text{"}$ woofer** right beside the indoor air-exit slot so the woofer cone and the air-exit slot are within **$12\text{ cm}$ ($5\text{ inches}$)** of each other. At $100\text{ Hz}$ ($\lambda = 343\text{ cm}$), two sources $12\text{ cm}$ apart fuse into a single acoustic point source!
4. **Microphone Mounting Distances:**
   - **Outdoor Reference Mic:** On a rigid $35\text{ cm}$ ($14\text{ in}$) rod extending outward from the outdoor intake grille. Wrap the capsule in open-cell foam + a faux-fur ("deadcat") wind muff.
   - **Indoor Error Mic:** On a small wire clip positioned **$25\text{ cm}$ ($10\text{ in}$) inside the room**, centered between the woofer cone and the ventilation exit slot.

---

## Part 3: Step-by-Step First Power-Up Checklist

1. **Flash the Firmware:** Open [`window_anc_fxlms.ino`](file:///data/data/com.termux/files/home/window-anc/firmware/window_anc_fxlms/window_anc_fxlms.ino) in Arduino IDE (with Teensyduino installed), select **Board: "Teensy 4.0"**, and upload.
2. **Set Amplifier Volume:** Start with the amplifier volume knob at **25%**.
3. **Run the 3-Second Auto-Calibration:**
   - Open the Arduino Serial Monitor ($115200\text{ baud}$) and press `c` (Recallibrate).
   - You will hear a soft 3-second burst of static (`shhhhh`) from the woofer. The indoor Error Mic records this burst to learn the exact delay and phase of your woofer.
4. **Verify Cancellation:**
   - Once calibration finishes, the Serial Monitor prints `Outdoor RMS`, `Indoor Error RMS`, and `Relative Level (dB)` every $0.5\text{ seconds}$.
   - Press `t` in the Serial Monitor to toggle between `ANC ON` and `BYPASS` so you can immediately hear the traffic bass drop out when ANC engages!
