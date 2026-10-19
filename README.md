# Open-Window Active Noise Control (ANC) System

**Authors:** `viaanshetty3-debug` & `lishirao-star`  
**License:** MIT

A complete hardware, firmware, and acoustic simulation project for blocking outdoor street traffic noise ($40\text{–}350\text{ Hz}$ bass rumble + high-frequency horns) from entering through an open window using a **Hybrid Passive Foam Window Baffle + Active Opposite-Phase (FxLMS) Woofer**.

---

## Project Structure

```text
window-anc/
├── firmware/
│   └── window_anc_fxlms/
│       └── window_anc_fxlms.ino        # Pro-level Teensy 4.0 C++ 128-tap Normalized Leaky FxLMS ISR (4,000 Hz)
├── kid_friendly_anc/
│   ├── simple_anc.ino                  # Easy-to-read 50-line ESP32 Wave-Flipper + Auto-Tuner firmware
│   └── simple_anc_demo.py              # 45-line Python demonstration of the 4-step wave cancellation loop
├── simulator/
│   ├── simulate_window_anc.py          # Synthetic traffic rumble FxLMS simulator (-18.7 dB cancellation)
│   ├── simulate_youtube_traffic.py     # Real-world Indian traffic audio test (-21.1 dB bass / -26.4 dB hybrid)
│   ├── simulate_realistic_hil.py       # 48 kHz 3D Bedroom + 12-bit Hardware-in-the-Loop (HIL) simulator
│   ├── output/                         # Playable WAV files from synthetic traffic test
│   ├── output_youtube/                 # Playable WAV files from real YouTube traffic test
│   └── output_realistic/               # 48 kHz binaural stereo WAVs & spectrum plot at 2.0m listener position
├── tablet_app/
│   └── index.html                      # Interactive Tablet/Browser WebAudio Live ANC Oscilloscope Lab
├── HARDWARE_AND_BAFFLE_GUIDE.md        # Complete pin-by-pin wiring schematic & window baffle build plans
└── LICENSE                             # MIT License
```

---

## How It Works (Plain English Summary)

1. **Passive S-Curve Foam Window Baffle ($> 350\text{ Hz}$):**
   High-frequency sounds (car horns, screeching brakes, voices) have tiny wavelengths and bounce around inside a foam-lined S-curve air channel, losing $15\text{–}24\text{ dB}$ of energy while fresh air still flows into the room.
2. **Active Opposite-Phase Woofer ($40\text{–}350\text{ Hz}$):**
   Low-frequency traffic rumble has giant wavelengths ($1.5\text{–}8\text{ meters}$) that pass right through foam. An outdoor **Reference Microphone** hears the incoming bass wave $35\text{–}38\text{ cm}$ early ($\sim 1.1\text{ ms}$ head-start). The microcontroller shapes an inverted wave ($180^\circ$ out of phase) across **128 time steps (`W[128]`)** and fires a **sealed $5.25\text{"}$ woofer** right as the bass wave exits the window slot—cancelling the bass before it spreads into the room.
3. **3-Second Startup Auto-Calibration (`S_hat` & `F_hat`):**
   On boot, the speaker plays 3 seconds of soft static to learn the exact delay of the speaker (`S_hat`) and how much sound leaks backward to the outdoor mic (`F_hat`).

---

## Quick Start

### 1. Run the Tablet Live Oscilloscope App
```bash
python3 -m http.server 8080 --directory tablet_app
```
Then open **`http://localhost:8080`** in Chrome on your tablet or laptop.

### 2. Run the Python Simulators
```bash
# Easy 45-line demonstration:
python3 kid_friendly_anc/simple_anc_demo.py

# Full 48 kHz 3D Bedroom & Hardware-in-the-Loop (HIL) simulation:
python3 simulator/simulate_realistic_hil.py
```
