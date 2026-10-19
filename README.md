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
| `simulate_window_anc.py` (toy) | −18.7 dB (synthetic rumble) | Ideal converters, **no analog filter delay**, hand-made acoustic paths |
| `simulate_youtube_traffic.py` (toy) | −21.1 dB bass band; "−26.4 dB hybrid" | Same toy assumptions, plus a baffle that is just an **assumed** 0.14 gain |
| `simulate_realistic_hil.py` (most detailed) | **≈ −7 to −9 dB** in 70–200 Hz, ≈ −6 dB full-band, at 2 m | 12-bit converters, analog-filter delay, 3D image-source room, non-linear woofer |
| `mismatch_sweep.py` | Closed-form limits | Gain error, timing error and causality of an *ideal* controller |

Take-aways from `python3 simulator/mismatch_sweep.py`:

* A 5 % gain error alone caps cancellation at −26 dB; 10 % caps it at −20 dB; 20 % at −14 dB.
* At 300 Hz, 50 µs of timing error caps it at about −20 dB, and 250 µs at about −7 dB. Timing is the binding constraint.
* **Delay decides a lot.** The acoustic head start is about 1.02 ms. The original RC-filter circuit plus the DAC hold adds about 1.45 ms, so the controller would have to predict the noise ~0.43 ms ahead (−5.7 dB for random 70–300 Hz noise with no prediction). Active 1 kHz Butterworth filters bring the total to about 0.58 ms, leaving ≈0.45 ms for ISR time, amplifier and speaker delay (calculated, not measured). `tools/timing_tool.py` redoes this with your measured numbers.
* Cancellation is local: the quiet zone is roughly $\lambda/10$ across, i.e. centimetres at 300 Hz. The HIL model's 2 m result relies on the woofer sitting within ~12 cm of the air slot.

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
│   └── timing_tool.py                  # Budget, measure ISR time over USB serial, change settings
├── tests/                              # test_feedback_alignment.py, test_timing_tool.py
├── tablet_app/index.html               # Tone/phase/timing lab (simulated speaker plane only)
└── HARDWARE_AND_BAFFLE_GUIDE.md        # Wiring, filter design, delay budget, baffle
```

## Quick start

```bash
pip install -r requirements.txt          # also needs ffmpeg for simulate_youtube_traffic.py
python3 simulator/mismatch_sweep.py      # what limits cancellation (fast, no audio)
python3 tools/timing_tool.py budget --preset butterworth-1k     # offline delay budget
python3 tools/timing_tool.py measure --port /dev/cu.usbmodemXXXX # real ISR time from the Teensy
python3 tools/timing_tool.py set --port /dev/cu.usbmodemXXXX mu 0.012   # also: anc on|off, leak, calibrate
python3 -m unittest discover -s tests -p "test_*.py"   # firmware-index and timing-tool tests
python3 simulator/simulate_realistic_hil.py   # most detailed model (about a minute)
python3 precision_anc/precision_phase_canceller.py [file.wav]
python3 precision_anc/live_tablet_server.py   # then open http://localhost:8080
# Tablet on the same Wi-Fi: add --host 0.0.0.0 (no authentication; trusted networks only)
```

## Known limitations

* The acoustic paths, baffle and glass in every simulator are modelled filters, not measurements.
* The firmware has not been compiled or run on a Teensy in this repo's history of testing; time the ISR (`micros()`) before trusting it.
* The delay budget is marginal; the analog filter, amplifier and woofer delays must be measured, and the ISR time read from the board (`timing_tool.py measure`, untested against real hardware).
* Wind noise, multiple noise directions, a moving listener and non-linear speaker behaviour at high excursion are only crudely modelled.
* Safety: calibration noise and divergence can be loud; start with the amplifier at minimum.

## Change log of this audit

Replaced the circular "zero-error" check; fixed a one-sample secondary-path offset in the HIL calibration (S_hat NMSE −6.1 → −20.2 dB), the image-source coordinates, and a one-sample feedback-neutralisation misalignment in the firmware; corrected the guide's RC-filter maths (−3 dB at 181 Hz, not 482 Hz; ~1 ms delay); implemented the documented button; hardened the tablet server; made the tablet demo stop playing `x + (−x)`.
