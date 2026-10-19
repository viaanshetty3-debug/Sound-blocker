# Open-Window Active Noise Control (ANC) System — `Sound-blocker`

**Authors:** `viaanshetty3-debug` & `lishirao-star`  
**License:** MIT

A complete hardware, firmware, and precision Python acoustic phase-cancellation system for blocking outdoor street traffic noise ($40\text{–}350\text{ Hz}$ bass rumble + high-frequency horns) from entering through an open window using a **Hybrid Passive Foam Window Baffle + Active Opposite-Phase ($180^\circ$) Speaker**.

---

## Project Structure

```text
Sound-blocker/
├── precision_anc/
│   ├── precision_phase_canceller.py    # Zero-Error Python Frequency/Phase Detector & Microsecond Timing Engine
│   ├── live_tablet_server.py           # Live HTTP + Python Backend connecting Tablet WebAudio to Python
│   └── output/                         # Verified zero-error (0.0000000000 RMS) opposite-phase WAV files
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

## Quick Start

### 1. Run the Precision Zero-Error Phase & Timing Calculator
Captures or loads incoming audio, detects exact frequencies ($\text{Hz}$), periods ($\text{ms}$), wavelengths ($\text{cm}$), and phase angles ($\phi^\circ$), calculates the exact microsecond hold time ($t_{\text{wait}} = t_{\text{acoustic}} - t_{\text{compute}} - t_{\text{hardware}}$), and applies a 64-tap Blackman-windowed Sinc fractional delay + $-1.000000\times$ phase inversion:

```bash
# Analyze default traffic audio & generate opposite-phase speaker WAV:
python3 precision_anc/precision_phase_canceller.py

# Record 3 seconds from live Android Tablet mic, calculate opposite phase, and play to speaker:
python3 precision_anc/precision_phase_canceller.py --record-mic --play
```

### 2. Launch the Live Tablet WebAudio + Python Server
```bash
python3 precision_anc/live_tablet_server.py
```
Then open **`http://localhost:8080`** in Chrome on your tablet or laptop.

### 3. Run the 3D Room & Hardware-in-the-Loop Simulators
```bash
# Easy 45-line demonstration:
python3 kid_friendly_anc/simple_anc_demo.py

# Full 48 kHz 3D Bedroom & Hardware-in-the-Loop (HIL) simulation:
python3 simulator/simulate_realistic_hil.py
```
