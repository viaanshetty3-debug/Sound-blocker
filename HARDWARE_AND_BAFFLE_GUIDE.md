# Open-Window Active Noise Control (ANC) — Hardware Schematic & Window Baffle Guide

This guide covers:
1. **Complete Pin-by-Pin Circuit Schematic** (Teensy 4.0 + Analog Low-Pass Filters + MCP4921 SPI DAC + Power Amp)
2. **Analog Anti-Aliasing & Reconstruction Filter Values** (and why their delay decides whether this can work at all)
3. **Mechanical Window Baffle Insert Design** (Hybrid passive high-frequency labyrinth + active bass-cancelling woofer core)

---

## Part 1: Electronic Circuit & Pinout Schematic

### Why This Signal Chain? (and the delay budget)
To cancel random outdoor traffic rumble, the total delay from microphone to speaker cone must be **shorter than the acoustic head start**. With the reference mic 35 cm from the speaker and the error mic 25 cm on the other side, that head start is $0.35\text{ m} / 343.235\text{ m/s} = 1019.7\text{ µs}$ at 20 °C (it changes with temperature). Standard USB/I2S audio boards use Sigma-Delta converters that delay audio by $1\text{–}5\text{ ms}$, so they cannot be used.

This design uses **SAR ADCs** and an **SPI DAC (`MCP4921`)**, but the converters are not the main delay. The analog filters are:

| Stage | Delay (low-frequency limit) |
|---|---|
| Original 2-section passive RC input filter, $R=1.5\text{ k}\Omega$, $C=220\text{ nF}$ (**do not use**) | $3RC = 990.0\text{ µs}$ (and its $-3\text{ dB}$ point is $180.49\text{ Hz}$, **not** 482 Hz: two cascaded RC sections do not give the single-section cutoff; unloaded formula) |
| 1-section RC reconstruction filter, same values | $RC \approx 0.33\text{ ms}$ |
| Zero-order hold at 4 kHz (half a sample period) | $0.125\text{ ms}$ |
| ISR execution | measure it: `python3 tools/timing_tool.py measure --port auto` (`--port virtual` tries the firmware without a board, with an assumed time) |
| **Known total, original circuit** (excludes ISR time, DAC settling, amplifier) | **$1445.0\text{ µs}$, against a head start of $1019.7\text{ µs}$ (20 °C): the controller must predict the noise at least $425.3\text{ µs}$ ahead** |

That is a soft limit, not a switch: tonal engine harmonics can be predicted, but for flat-spectrum 70–300 Hz noise, lateness of $425.3\text{ µs}$ alone leaves at best $-5.73\text{ dB}$ (exact formula in `simulator/mismatch_sweep.py`, no prediction assumed). Use active 2nd-order Butterworth (Sallen-Key) filters instead: $225.1\text{ µs}$ each at $f_c = 1\text{ kHz}$, so $575.2\text{ µs}$ known in total with the hold, leaving **at most** $444.6\text{ µs}$ for the ISR, DAC settling, amplifier and anything else. `python3 tools/timing_tool.py budget --preset original-rc` (or `butterworth-1k`) prints the arithmetic and accepts your own measured delays. Raising the sample rate shrinks the hold term (1/(2·fs)) and lets the filters move higher. For example, a single-pole RC (3.3 kΩ, 10 nF, $f_c = 4823\text{ Hz}$, $33.0\text{ µs}$ each) at 48 kHz has a known delay of $76.4\text{ µs}$, but it attenuates the 47.7 kHz content that aliases onto 300 Hz by only $19.948\text{ dB}$, and a 128-tap filter would then span $2.667\text{ ms}$ instead of $32\text{ ms}$. It also needs the firmware's `SAMPLE_RATE_HZ`, tap counts and ISR time re-checked; the whole ISR must finish within one sample period ($20.8\text{ µs}$ at 48 kHz). **None of these numbers has been measured on a built board**; measure the real mic-to-speaker delay with a scope and the ISR time with `timing_tool.py measure`, and feed both back into `timing_tool.py budget`.

### Master Wiring Diagram

```
====================================================================================
1. MICROPHONE INPUT STAGE (Build 2 Identical Channels: Outdoor Ref & Indoor Error)
====================================================================================

 [Electret mic board] ---> [Sallen-Key 2nd-order Butterworth, fc = 1 kHz] ---> Teensy ADC
  (e.g. Adafruit 1063,         unity gain, referenced to AGND = 1.65 V         - Outdoor Ref -> A0 (14)
   MAX4466 board: fixed        (rail-to-rail op-amp, e.g. MCP6022, 3.3 V)       - Indoor Err  -> A1 (15)
   gain set by its trimmer,
   output rests at VCC/2,    in --[R1 = 11k]--A--[R2 = 11k]--B--(+) op-amp
   DC-coupled)                                |                |   (-) tied to output ---> out
                                          [C1 = 20 nF]     [C2 = 10 nF]
                                          A to op-amp      B to AGND
                                          output
  * C1 = two 10 nF capacitors in parallel; C2 = one 10 nF. Use film or C0G capacitors.
  * fc = 1 / (2*pi*R*sqrt(C1*C2)) = 1 / (2*pi*11k*14.14 nF) = 1.023 kHz;  Q = 0.5*sqrt(C1/C2) = 0.7071
  * With these exact values the cutoff is 1023.1 Hz (not exactly 1 kHz) and the DC group delay is
    C2*(R1+R2) = 10 nF * 22 kohm = 220.0 us (calculated, ideal op-amp, not measured).
  * At Fs = 4 kHz the filter attenuates the 2 kHz Nyquist frequency by only 11.93 dB, so some
    aliasing of horns/hiss remains. That is a trade-off against delay.

  ANALOG GROUND (AGND = 1.65 V). On a single 3.3 V supply the filter capacitors must return to a
  stable 1.65 V, not to 0 V, and the signal rests at 1.65 V:

    3.3 V --[10k]--+--[10k]-- GND          (two 10k resistors, 1.65 V)
                   |
                   +--[100 nF to GND]--(+) spare op-amp section, (-) tied to its output = AGND
                                        and a 10 uF capacitor from AGND to GND

  Four op-amp sections are needed in total: three filters (reference mic, error mic, output)
  plus this AGND buffer, so two dual op-amp chips. Do NOT bias a filter by hanging a divider on
  its (+) input: that resistance sits in parallel with C2 and changes the filter.

  The MAX4466 board's output is already biased at VCC/2 (Adafruit's product page), so connect it
  directly (no coupling capacitor). Power the board from the Teensy's 3.3 V so that bias is 1.65 V.

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
                                   [same Sallen-Key Butterworth, fc = 1 kHz, referenced to AGND]
                                        |  (the DAC output rests at 1.65 V at mid-code, so it is
                                        |   DC-coupled into the filter)
                                        |  (reconstruction filter, 220.0 us)
                                   [C_dc = 10 uF Film/Electrolytic Cap] (blocks the 1.65 V DC, after the filter)
                                        |
                                        +---> To Power Amp Analog IN

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
- **AGC caveat:** the MAX9814 has a built-in Automatic Gain Control that varies its gain with level. The adaptive filter assumes a fixed, linear mic path, so AGC works against it. I have not confirmed that the MAX9814 can be run at truly fixed gain (check its datasheet); a fixed-gain op-amp preamp for the electret is the safer choice.

---

## Part 2: The Window Baffle Mechanical Design (Hybrid Active + Passive)

### Why You Need a Window Insert Box
If a window is wide open ($60\text{+}\text{ cm}$), high-frequency street noise ($500\text{–}4000\text{ Hz}$) and wind rush right past the speaker. By building a removable **Window Baffle Insert** (similar to a portable air conditioner window panel, made from $12\text{ mm}$ / $1/2\text{"}$ plywood or MDF), you combine **two technologies**:
1. **Passive acoustic labyrinth (intended for $> 350\text{ Hz}$):** A foam-lined S-curve air channel lets air through while absorbing some tire hiss, voices and sirens. How much it absorbs is **not measured** here; thin foam does little at low frequencies, and the simulators only assume values for it.
2. **Active bass woofer (intended for roughly $70\text{–}300\text{ Hz}$):** Low-frequency bass passes through the foam channel, where the **sealed $5.25\text{"}$ woofer** fires an inverted wave near the duct exit. The HIL simulator predicts only about $7\text{–}9\text{ dB}$ of reduction in the core bass band, limited by causality, not the roughly $20\text{ dB}$ of the idealised toy models.

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
   - Mount the **$5.25\text{"}$ woofer** right beside the indoor air-exit slot so the woofer cone and the air-exit slot are within **$12\text{ cm}$ ($5\text{ inches}$)** of each other. At $100\text{ Hz}$ ($\lambda = 343\text{ cm}$) two sources $12\text{ cm}$ apart behave approximately like one point source; by $300\text{ Hz}$ ($\lambda \approx 114\text{ cm}$) the spacing is about $\lambda/10$ and the approximation is getting poor.
4. **Microphone Mounting Distances:**
   - **Outdoor Reference Mic:** On a rigid $35\text{ cm}$ ($14\text{ in}$) rod extending outward from the outdoor intake grille. Wrap the capsule in open-cell foam + a faux-fur ("deadcat") wind muff.
   - **Indoor Error Mic:** On a small wire clip positioned **$25\text{ cm}$ ($10\text{ in}$) inside the room**, centered between the woofer cone and the ventilation exit slot.

---

## Part 3: Step-by-Step First Power-Up Checklist

1. **Flash the Firmware:** Open [`window_anc_fxlms.ino`](firmware/window_anc_fxlms/window_anc_fxlms.ino) in Arduino IDE (with Teensyduino installed), select **Board: "Teensy 4.0"**, and upload.
2. **Set Amplifier Volume:** Start with the volume knob at its **minimum** and raise it slowly. The calibration burst and any divergence can be loud; keep the speaker away from your ears and do not leave the system running unattended. Use an amplifier rated for the woofer and fuse it appropriately.
3. **Run the 3-Second Auto-Calibration:**
   - Open the Arduino Serial Monitor ($115200\text{ baud}$) and press `c` (Recallibrate).
   - You will hear a soft 3-second burst of static (`shhhhh`) from the woofer. The indoor Error Mic records this burst to learn the exact delay and phase of your woofer.
4. **Verify Cancellation:**
   - Once calibration finishes, the Serial Monitor prints `Outdoor RMS`, `Indoor Error RMS` and `Error/Ref level` every $0.5\text{ seconds}$.
   - `Error/Ref level` compares two different microphones and is not an attenuation figure. Instead, toggle `ANC ON` and `BYPASS` (`t`, or the button on pin 2) and compare the **`Indoor Error RMS`** between the two states over several seconds of similar traffic.
   - Before expecting any cancellation, check the delay budget above with an oscilloscope and `tools/timing_tool.py measure`: if mic-to-speaker delay exceeds the acoustic head start, the controller must predict the noise and broadband reduction collapses.
