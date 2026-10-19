#!/usr/bin/env python3
"""
Mismatch & causality sweep: how much can feed-forward cancellation really remove?
=================================================================================
Replaces the old "zero-error" check, which was circular (the anti-noise and the
noise were built from the same delayed array, so they cancelled for any input).

Here the controller is an ideal, perfectly-converged feed-forward canceller that
is only allowed what real hardware has:
  * a gain error g on the anti-noise (speaker/amp/mic sensitivity drift),
  * a timing error dt (analog filter delay, ISR sampling, ZOH, speed-of-sound
    drift), and
  * CAUSALITY: the anti-noise for a broadband signal can only use what the
    reference mic has already heard. If the electronics are slower than the
    acoustic head start, the controller must predict the noise ahead, which
    works for tonal content and degrades quickly for random broadband noise.

Part 1  tone residual  |1 - g*exp(-j*2*pi*f*dt)|   (closed form, checked below)
Part 2  broadband residual: band-limited noise, anti-noise delayed by dt, via
        sample-accurate FFT delay, measured from actual signals.
Part 3  delay budget of the documented 35 cm / 25 cm geometry (shared with tools/timing_tool.py).

Run:  python3 simulator/mismatch_sweep.py
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
import timing_model as tm  # noqa: E402

C_SOUND = 343.0
FS = 48000


def tone_residual_db(freq_hz, gain_err=1.0, delay_err_s=0.0):
    """Residual of cancelling a pure tone with gain g and extra delay dt (dB re uncancelled)."""
    w = 2.0 * np.pi * freq_hz
    return max(-60.0, 20.0 * np.log10(np.abs(1.0 - gain_err * np.exp(-1j * w * delay_err_s)) + 1e-300))


def fft_delay(x, delay_s, fs=FS):
    """Circular fractional delay by phase ramp (exact for band-limited periodic x)."""
    n = len(x)
    f = np.fft.rfftfreq(n, 1.0 / fs)
    return np.fft.irfft(np.fft.rfft(x) * np.exp(-2j * np.pi * f * delay_s), n)


def bandlimited_noise(f_lo, f_hi, seconds=8.0, seed=0, fs=FS):
    rng = np.random.default_rng(seed)
    n = int(seconds * fs)
    spec = np.fft.rfft(rng.standard_normal(n))
    f = np.fft.rfftfreq(n, 1.0 / fs)
    spec[(f < f_lo) | (f > f_hi)] = 0.0
    x = np.fft.irfft(spec, n)
    return x / np.sqrt(np.mean(x ** 2))


def broadband_residual_db(x, gain_err, delay_err_s):
    res = x - gain_err * fft_delay(x, delay_err_s)
    return max(-60.0, 20.0 * np.log10(np.sqrt(np.mean(res ** 2)) / np.sqrt(np.mean(x ** 2)) + 1e-300))


def main():
    print("=" * 74)
    print(" FEED-FORWARD CANCELLATION vs. REAL-WORLD ERRORS (ideal controller)")
    print("=" * 74)

    # Self-check of the closed form against the signal-domain measurement
    t = np.arange(FS) / FS
    tone = np.sin(2 * np.pi * 100.0 * t)
    sig = 20 * np.log10(np.sqrt(np.mean((tone - 0.95 * fft_delay(tone, 100e-6)) ** 2)) / np.sqrt(np.mean(tone ** 2)))
    ana = tone_residual_db(100.0, 0.95, 100e-6)
    assert abs(sig - ana) < 0.05, (sig, ana)
    print(f" [check] closed form vs simulated tone: {ana:.2f} dB vs {sig:.2f} dB  OK")
    xb = bandlimited_noise(70.0, 300.0)
    sim_b = broadband_residual_db(xb, 1.0, 250e-6)
    ana_b = tm.band_residual_db(70.0, 300.0, 250e-6)
    assert abs(sim_b - ana_b) < 0.3, (sim_b, ana_b)
    print(f" [check] flat 70-300 Hz noise, 250 us late: exact formula {ana_b:.2f} dB, simulated noise {sim_b:.2f} dB (random noise: small scatter)\n")

    print(" Part 1 - single tone, residual in dB (more negative = better)")
    delays_us = [0, 10, 50, 100, 250, 500]
    gains = [1.00, 0.95, 0.90, 0.80]
    for f in (50, 100, 200, 300):
        print(f"\n   {f} Hz tone      timing error ->" + "".join(f"{d:>8d}us" for d in delays_us))
        for g in gains:
            row = "".join(f"{tone_residual_db(f, g, d * 1e-6):>10.1f}" for d in delays_us)
            print(f"     gain error {100 * (g - 1):>+5.0f}%          {row}")

    print("\n Part 2 - broadband noise (70-300 Hz), measured from signals")
    x = bandlimited_noise(70.0, 300.0)
    print("   timing error ->" + "".join(f"{d:>8d}us" for d in delays_us))
    for g in gains:
        row = "".join(f"{broadband_residual_db(x, g, d * 1e-6):>10.1f}" for d in delays_us)
        print(f"   gain {100 * (g - 1):>+4.0f}%        {row}")
    print("   Rule of thumb confirmed: to reach -20 dB you need gain within ~10% AND")
    print("   phase within ~6 deg at the highest frequency (300 Hz: ~55 us!).")

    print("\n Part 3 - delay budget of the documented geometry (35 cm ref->speaker, 25 cm speaker->error)")
    print(" (same model as tools/timing_tool.py; terms that need a measurement are shown as NOT SUPPLIED)")
    for name in ("original-rc", "butterworth-1k"):
        in_f, out_f = tm.PRESETS[name]
        bud = tm.compute_budget(in_filter=in_f, out_filter=out_f)
        print()
        print(tm.format_budget(bud, f"preset {name}"))
    in_f, out_f = tm.PRESETS["original-rc"]
    bud = tm.compute_budget(in_filter=in_f, out_filter=out_f)
    short = -bud["margin_upper_bound_s"]
    best = tm.band_residual_db(70.0, 300.0, short)
    print("\n   A shortfall is a SOFT limit: the controller must predict the noise ahead.")
    print(f"   For noise with a flat spectrum over 70-300 Hz, a pure lateness of {short * 1e6:.1f} us (the minimum for the")
    print(f"   original circuit; unmeasured terms only add to it) leaves at best {best:.2f} dB (exact formula, no prediction).")
    print("   Tonal engine harmonics are predictable and can do better; random tyre noise cannot.")
    print("=" * 74)


if __name__ == "__main__":
    main()
