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
    acoustic head start, the controller must predict the future, which no
    filter can do for noise.

Part 1  tone residual  |1 - g*exp(-j*2*pi*f*dt)|   (closed form, checked below)
Part 2  broadband residual: band-limited noise, anti-noise delayed by dt, via
        sample-accurate FFT delay, measured from actual signals.
Part 3  causality budget of the documented 35 cm / 25 cm geometry.

Run:  python3 simulator/mismatch_sweep.py
"""

import numpy as np

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
    print(f" [check] closed form vs simulated tone: {ana:.2f} dB vs {sig:.2f} dB  OK\n")

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

    print("\n Part 3 - causality budget of the documented geometry")
    ref_to_spk = 0.35 / C_SOUND
    spk_to_err = 0.25 / C_SOUND
    head_start = ref_to_spk  # time from ref mic hearing it to wavefront passing the speaker
    print(f"   acoustic head start (35 cm ref mic -> speaker)      : {head_start * 1e3:6.3f} ms")
    print(f"   speaker -> error mic (25 cm)                        : {spk_to_err * 1e3:6.3f} ms")
    margin = (0.35 + 0.25) / C_SOUND - spk_to_err
    print("   cancellation happens at the error mic, so the time left for electronics is")
    print(f"   (ref->err acoustic path) - (speaker->err path)      : {margin * 1e3:6.3f} ms")
    rc2 = 3 * 1500 * 220e-9
    rc1 = 1500 * 220e-9
    items = [
        ("input 2-section passive RC (3RC, original guide)", rc2),
        ("output 1-section RC reconstruction (RC)", rc1),
        ("ZOH + ISR sampling (1.5 sample @4kHz)", 1.5 / 4000.0),
    ]
    total = 0.0
    for name, d in items:
        total += d
        print(f"   - {name:<52}: {d * 1e3:6.3f} ms")
    print(f"   total electronics delay (original circuit)          : {total * 1e3:6.3f} ms")
    print(f"   => margin left: {1e3 * (margin - total):+.3f} ms  "
          f"({'CAUSAL' if margin - total > 0 else 'NON-CAUSAL: cannot cancel broadband noise'})")

    sk = 1.4142 / (2 * np.pi * 1000.0)
    total2 = sk + 1.5 / 4000.0 + 1.4142 / (2 * np.pi * 1000.0)
    print(f"\n   with 2nd-order Butterworth fc=1 kHz in + out (~{sk * 1e3:.2f} ms each) : total {total2 * 1e3:6.3f} ms"
          f"  margin {1e3 * (margin - total2):+.3f} ms")
    print("   (group delay approximations at low frequency; a higher ISR rate, e.g. 8-16 kHz,")
    print("    shrinks the ZOH/ISR term and lets the anti-alias filters move higher.)")
    print("=" * 74)


if __name__ == "__main__":
    main()
