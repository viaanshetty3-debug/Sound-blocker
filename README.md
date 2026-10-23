# Open-Window Active Noise Control (ANC) — `Sound-blocker`

**Authors:** `viaanshetty3-debug` & `lishirao-star`  
**License:** MIT

An experimental hardware, firmware and Python simulation project for reducing low-frequency street-traffic noise ($\approx 70\text{–}300\text{ Hz}$) coming through an open window, using a **foam-lined window baffle plus an active feed-forward (FxLMS) speaker**.

> **Status: simulation and unbuilt design. Nothing here has been measured on a real window.**  
> The numbers below come from models, and the more realistic the model, the smaller the benefit. Read "What the simulations say" before quoting any figure.

---

## What the simulations say

| Model | Noise reduction | Why it differs |
|---|---|---|
| `simulate_window_anc.py` (toy) | −18.7 dB (steady state, t = 6–10 s, synthetic rumble) | Ideal converters, **no analog filter delay**, hand-made acoustic paths |
| `simulate_youtube_traffic.py` (toy) | −21.1 dB bass band (<320 Hz); −26.4 dB "hybrid" total | Same toy assumptions, plus a baffle that is just an **assumed** 0.14 gain |
| `simulate_realistic_hil.py` (most detailed) | At the listener, 2 m: −9.2 dB in 70–200 Hz (baffle + ANC vs. bare window); ANC alone −7.8 dB in 70–220 Hz; −6.3 dB full-band | 12-bit converters, analog-filter delay, 3D image-source room, non-linear woofer |
| `mismatch_sweep.py` | Exact formulas (below) | Gain error, timing error and delay budget of an *ideal* controller |

The three simulator rows are the values the current code prints with its fixed random seeds. They describe the models, not a real window.

**ADC resolution.** The Teensy 4.0 reads 12-bit codes, but PJRC states that only about 10 bits are normally usable because of noise. The HIL model's default ADC noise (12-bit quantisation plus 1.2 LSB rms) is an ENOB of 9.90 bits. Sweeping it with `--adc-enob` leaves every figure above unchanged at 0.1 dB resolution from 12 bits down to 8 bits (S_hat NMSE −20.2 dB throughout). At 6 bits the listener figure moves by 0.1 dB, and at 4 bits ANC alone at the listener is −7.4 dB instead of −7.8 dB. So in this model, ADC noise is not what limits cancellation. This holds only at the model's signal level: the street recording is scaled to a 0.85 peak, close to full scale. A real microphone preamp set to a lower level would lose more of its bits to noise, and that has not been simulated.

Take-aways from `python3 simulator/mismatch_sweep.py`:

* Gain error alone limits a tone's cancellation to 20·log10(error): a 5 % error gives −26.0 dB, 10 % gives −20.0 dB, 20 % gives −14.0 dB.
* Timing error τ limits it to 20·log10(2·sin(π f τ)): at 300 Hz, 50 µs gives −20.5 dB and 250 µs gives −6.6 dB.
* **Delay budget** (speed of sound 343.235 m/s at 20 °C, mic 35 cm from the speaker, head start H = 1019.7 µs). Only the terms that can be calculated are included; the ISR time, DAC settling and amplifier delay still need to be measured, and each can only add delay, so these are bounds:

  | Filters (fs = 4 kHz unless stated) | Known delay | Margin | Break-even distance |
  |---|---|---|---|
  | `original-rc` (guide v1) | 1445.0 µs | ≤ −425.3 µs | ≥ 49.60 cm |
  | `butterworth-1k` | 575.2 µs | ≤ +444.6 µs | ≥ 19.74 cm |
  | `sk-11k-20n-10n` (the parts in the guide: 11 kΩ, 20 nF, 10 nF; fc = 1023.1 Hz) | 565.0 µs | ≤ +454.7 µs | ≥ 19.39 cm |
  | `rc1-3k3-10n`, fs = 48 kHz | 76.4 µs | ≤ +943.3 µs | ≥ 2.62 cm |

  For `original-rc` the controller must predict the noise **at least** 425.3 µs ahead. For flat-spectrum 70–300 Hz noise that is lateness alone leaves at best −5.73 dB (exact formula, no prediction; tonal content does better). The other two rows are undetermined until the missing terms are measured. `rc1-3k3-10n` attenuates 47.7 kHz, the frequency that aliases onto 300 Hz, by only 19.948 dB, and at 48 kHz a 128-tap filter spans 2.667 ms instead of 32 ms. `python3 tools/timing_tool.py budget` reproduces this table.
* Cancellation is local. A common rule of thumb (not derived here) is that useful reduction extends over a region roughly one tenth of a wavelength across, which is 11.4 cm at 300 Hz (343.235 m/s). The HIL model places the woofer 12 cm from the air slot; that spacing is an assumption of the model, not a tested requirement.

What is **not** claimed anywhere in this repo: zero-error cancellation, real-world validated performance, or a measured baffle insertion loss. (An earlier version claimed "0.0000000000 RMS / 100 % cancelled"; that check subtracted a signal from itself and was removed. `precision_phase_canceller.py` now reports the residual under stated gain/timing errors and labels the perfect-path value as zero by construction.)

---

## Project structure

```text
Sound-blocker/
├── precision_anc/
│   ├── precision_phase_canceller.py    # Tone detection, timing budget, anti-noise with stated gain/timing errors
│   ├── live_tablet_server.py           # Local HTTP server for the tablet page (127.0.0.1 by default)
│   └── output/                         # Generated WAVs
├── firmware/window_anc_fxlms/
│   └── window_anc_fxlms.ino            # Teensy 4.x 128-tap normalized leaky FxLMS (4 kHz) - NOT compiled/tested on hardware here
├── kid_friendly_anc/                   # Teaching demos (single steady tone)
├── simulator/
│   ├── mismatch_sweep.py               # Gain / timing / delay-budget limits (start here)
│   ├── simulate_realistic_hil.py       # Most detailed model
│   ├── simulate_window_anc.py          # Toy FxLMS model
│   ├── simulate_youtube_traffic.py     # Toy model on a traffic recording
│   └── input_youtube.wav               # Third-party recording; confirm you may use/redistribute it
├── tools/
│   ├── timing_model.py                 # Delay-budget arithmetic (shared)
│   ├── timing_tool.py                  # Budget, measure ISR time over USB serial, change settings
│   ├── virtual_teensy.py               # Runs the real firmware on your computer (no board needed)
│   └── virtual_teensy/                 # Host stubs of the Teensy core + harness with a toy plant
├── tests/                              # firmware indexing, timing tool, ADC model, virtual Teensy
├── tablet_app/index.html               # Tone/phase/timing lab (simulated speaker plane only)
└── HARDWARE_AND_BAFFLE_GUIDE.md        # Wiring, filter design, delay budget, baffle
```

## Quick start

```bash
pip install -r requirements.txt          # also needs ffmpeg for simulate_youtube_traffic.py
python3 simulator/mismatch_sweep.py      # what limits cancellation (fast, no audio)
python3 tools/timing_tool.py budget                            # offline delay budget (guide's filters)
python3 tools/timing_tool.py measure --port virtual            # no board: real firmware, ASSUMED timing
python3 tools/timing_tool.py measure --port auto               # real ISR time from the plugged-in Teensy
python3 tools/timing_tool.py set --port auto mu 0.012          # also: anc on|off, leak, calibrate
python3 -m unittest discover -s tests -p "test_*.py"   # all tests (the virtual-Teensy ones need a C++ compiler)
python3 simulator/simulate_realistic_hil.py   # most detailed model (a few minutes); --adc-enob N, --no-output
python3 precision_anc/precision_phase_canceller.py [file.wav]
python3 precision_anc/live_tablet_server.py   # then open http://localhost:8080
# Tablet on the same Wi-Fi: add --host 0.0.0.0 (no authentication; trusted networks only)
```

## Virtual Teensy (no board needed)

`--port virtual` compiles the unmodified `window_anc_fxlms.ino` for your computer against small stand-ins for the Teensy libraries and runs its 4,000 Hz interrupt in real time. Every serial command (`params`, `set`, `calibrate`, `measure`) talks to the firmware's real code, so protocol bugs show up without hardware. It found one: `calibrate` used to start a calibration through the single-key `c` shortcut and then answer `@ERR unknown command`. That is now fixed.

It cannot tell you how fast the firmware is on a Teensy, because your computer's speed says nothing about a 600 MHz Cortex-M7. Its cycle counter reports an **assumed** ISR time (`--virtual-isr-us`, default 250 µs, the most the ISR may take without losing a sample). The tool labels that term `ASSUMED` and never gives a passing verdict on it. With the default it prints the worst case for the guide's filters, d_min ≥ 27.97 cm (the DAC and amplifier are still unknown). The microphones hear a toy plant (two tones plus noise through pure delays), so the dB levels it prints say nothing about a real window. `python3 tools/virtual_teensy.py --pty` exposes it as a serial port for other programs. It needs a C++ compiler (on a Mac: `xcode-select --install`).

## Known limitations

* The acoustic paths, baffle and glass in every simulator are modelled filters, not measurements.
* The firmware has not been compiled or run on a Teensy in this repo's history of testing. It does compile and run on a computer (the virtual Teensy), which checks its logic but not its timing.
* The delay budget is marginal; the analog filter, amplifier and woofer delays must be measured, and the ISR time read from the board (`timing_tool.py measure`, untested against real hardware).
* Wind noise, multiple noise directions, a moving listener and non-linear speaker behaviour at high excursion are only crudely modelled.
* Safety: calibration noise and divergence can be loud; start with the amplifier at minimum.

## Change log of this audit

Replaced the circular "zero-error" check; fixed a one-sample secondary-path offset in the HIL calibration (S_hat NMSE −6.1 → −20.2 dB), the image-source coordinates, and a one-sample feedback-neutralisation misalignment in the firmware; corrected the guide's RC-filter maths (the two-section filter's −3 dB point is 180.49 Hz, not 482 Hz, and its delay is 990 µs); implemented the documented button; hardened the tablet server; made the tablet demo stop playing `x + (−x)`.
