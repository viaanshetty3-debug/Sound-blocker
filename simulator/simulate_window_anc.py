#!/usr/bin/env python3
"""
Open-Window Active Noise Control (ANC) Acoustic Simulator
=========================================================
Simulates the geometry and the two-stage Normalized Leaky FxLMS algorithm used in
`window_anc_fxlms.ino` at Fs = 4,000 Hz.

TOY MODEL - read before quoting any number from this file
---------------------------------------------------------
Ideal converters (no quantization), NO analog anti-alias/reconstruction filter delay,
a single plane-wave direction, and acoustic paths P/S/F that are HAND-MADE impulse
responses (not measured). Under these assumptions the controller is causal with
~1 ms to spare, which real hardware with RC filters does not have (see
simulator/mismatch_sweep.py and simulate_realistic_hil.py, which gives about -7 dB).
Use this file to study the FxLMS algorithm, not to predict window performance.

Physical Geometry Modeled:
- Outdoor Reference Mic -> Speaker ( Window Plane ): 35 cm (~1.02 ms)
- Speaker ( Window Plane ) -> Indoor Error Mic     : 25 cm (~0.73 ms)
- Total Primary Path (Ref Mic -> Error Mic)        : 60 cm (~1.75 ms)
- Sealed 5.25" Woofer + Amplifier Phase Response   : Modeled in S(z)
- Cardioid Mic Acoustic Feedback (Speaker -> Ref)  : Modeled in F(z)

Outputs playable WAV files in `./output/`:
- `1_room_anc_off.wav`         : Uncancelled traffic rumble inside the room
- `2_room_anc_on.wav`          : Cancelled residual sound after FxLMS converges
- `3_demo_off_then_on.wav`     : 4s ANC OFF followed by 6s ANC ON (hear it live!)
"""

import math
import os
import random
import struct
import wave
from array import array

# ============================================================================
# 1. SIMULATION PARAMETERS (Matching Teensy 4.0 Firmware)
# ============================================================================
FS = 4000                       # Sample rate: 4,000 Hz
W_TAPS = 128                    # Control filter length
S_TAPS = 64                     # Secondary path model length

MU_S_IDENT = 0.08               # Calibration step size
MU_FXLMS = 0.015                # Active cancellation step size
LEAKAGE = 0.99995               # Leaky LMS decay factor
EPSILON = 1e-4                  # Normalization regularization floor


def dot_product(a, b, n):
    """Fast dot product of two arrays of length n."""
    s = 0.0
    for i in range(n):
        s += a[i] * b[i]
    return s


def save_wav(filename, samples, fs=FS):
    """Save normalized float samples [-1.0, 1.0] to a 16-bit PCM WAV file."""
    os.makedirs(os.path.dirname(filename), exist_ok=True)
    pcm = array("h")
    for s in samples:
        clipped = max(-0.98, min(0.98, s))
        pcm.append(int(clipped * 32767.0))
    with wave.open(filename, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(fs)
        wf.writeframes(pcm.tobytes())


# ============================================================================
# 2. ACOUSTIC PATH MODELS (Primary P(z), Secondary S(z), Feedback F(z))
# ============================================================================
def build_acoustic_paths():
    """
    Construct realistic impulse responses for:
    - P(z): Primary path from Outdoor Ref Mic through open window to Error Mic
            (60 cm air delay = 7 samples @ 4 kHz + window diffraction/room modes)
    - S(z): Secondary path from Sealed Woofer to Indoor Error Mic
            (25 cm air delay = 3 samples @ 4 kHz + woofer enclosure response)
    - F(z): Acoustic feedback from Sealed Woofer back to Outdoor Ref Mic
            (35 cm air delay = 4 samples @ 4 kHz, attenuated by cardioid mic null)
    """
    P_true = [0.0] * W_TAPS
    S_true = [0.0] * S_TAPS
    F_true = [0.0] * S_TAPS

    # Primary Path P(z): 7-sample acoustic time-of-flight + damped room reflections
    P_true[7] = 0.82
    P_true[8] = 0.45
    P_true[9] = -0.18
    P_true[10] = -0.12
    for k in range(11, 55):
        t = k - 7
        P_true[k] = 0.22 * math.exp(-0.11 * t) * math.cos(0.42 * t)

    # Secondary Path S(z): 3-sample time-of-flight + sealed woofer phase shift
    S_true[3] = 0.75
    S_true[4] = 0.52
    S_true[5] = -0.28
    S_true[6] = -0.19
    for k in range(7, 36):
        t = k - 3
        S_true[k] = 0.18 * math.exp(-0.16 * t) * math.sin(0.55 * t)

    # Acoustic Feedback F(z): 4-sample time-of-flight, attenuated ~14 dB by cardioid mic
    F_true[4] = 0.16
    F_true[5] = 0.09
    F_true[6] = -0.06
    for k in range(7, 25):
        t = k - 4
        F_true[k] = 0.04 * math.exp(-0.20 * t) * math.cos(0.50 * t)

    return P_true, S_true, F_true


# ============================================================================
# 3. SYNTHETIC LOW-FREQUENCY TRAFFIC BASS & RUMBLE GENERATOR (40 - 220 Hz)
# ============================================================================
def generate_traffic_rumble(num_samples, seed=42):
    """
    Generates realistic outdoor urban traffic noise:
    - Diesel / truck engine fundamental & harmonics (52 Hz, 78 Hz, 104 Hz, 145 Hz)
    - Slow amplitude modulation (passing vehicles)
    - Broadband low-frequency tire/road rumble (filtered 45 - 210 Hz random noise)
    """
    rng = random.Random(seed)
    signal = [0.0] * num_samples

    # State for 2-pole low-pass + high-pass road rumble filter
    lp1 = 0.0
    lp2 = 0.0
    hp_prev_in = 0.0
    hp_prev_out = 0.0

    for n in range(num_samples):
        t = n / float(FS)

        # Passing vehicle envelope modulation
        env = 0.75 + 0.25 * math.sin(2.0 * math.pi * 0.35 * t)

        # Multi-tone engine hum + exhaust bass
        engine = (
            0.32 * math.sin(2.0 * math.pi * 52.0 * t + 0.1 * math.sin(2.0 * math.pi * 0.5 * t))
            + 0.24 * math.sin(2.0 * math.pi * 78.0 * t)
            + 0.18 * math.sin(2.0 * math.pi * 104.0 * t + 0.8)
            + 0.12 * math.sin(2.0 * math.pi * 148.0 * t + 1.7)
        )

        # Broadband tire/asphalt rumble (band-limited 45 - 200 Hz)
        white = rng.uniform(-1.0, 1.0)
        lp1 = lp1 + 0.18 * (white - lp1)
        lp2 = lp2 + 0.18 * (lp1 - lp2)
        road_rumble = 0.96 * (hp_prev_out + lp2 - hp_prev_in)
        hp_prev_in = lp2
        hp_prev_out = road_rumble

        signal[n] = env * (engine + 0.45 * road_rumble)

    return signal


# ============================================================================
# 4. TWO-STAGE NORMALIZED LEAKY FxLMS SIMULATION
# ============================================================================
def run_simulation():
    print("=" * 70)
    print(" OPEN-WINDOW ACTIVE NOISE CONTROL (ANC) — ACOUSTIC SIMULATOR")
    print("=" * 70)

    P_true, S_true, F_true = build_acoustic_paths()

    # Adaptive filter weights (initialized to zero, just like the Teensy firmware)
    W = [0.0] * W_TAPS
    S_hat = [0.0] * S_TAPS
    F_hat = [0.0] * S_TAPS

    # ------------------------------------------------------------------------
    # STAGE 1: 3-Second Offline Secondary Path Calibration (S_hat & F_hat)
    # ------------------------------------------------------------------------
    calib_samples = int(3.0 * FS)
    y_hist = [0.0] * S_TAPS
    rng_calib = random.Random(12345)

    for _ in range(calib_samples):
        v_n = rng_calib.uniform(-0.18, 0.18)
        y_hist.insert(0, v_n)
        y_hist.pop()

        # True acoustic response at Error Mic and Reference Mic
        e_n = dot_product(S_true, y_hist, S_TAPS) + rng_calib.gauss(0.0, 0.001)
        x_n = dot_product(F_true, y_hist, S_TAPS) + rng_calib.gauss(0.0, 0.001)

        # Model predictions
        e_hat = dot_product(S_hat, y_hist, S_TAPS)
        f_hat = dot_product(F_hat, y_hist, S_TAPS)
        power_v = dot_product(y_hist, y_hist, S_TAPS) + EPSILON

        step_s = (MU_S_IDENT * (e_n - e_hat)) / power_v
        step_f = (MU_S_IDENT * (x_n - f_hat)) / power_v
        for k in range(S_TAPS):
            S_hat[k] += step_s * y_hist[k]
            F_hat[k] += step_f * y_hist[k]

    # Measure Secondary Path modeling accuracy
    s_err_pow = sum((S_true[k] - S_hat[k]) ** 2 for k in range(S_TAPS))
    s_true_pow = sum(S_true[k] ** 2 for k in range(S_TAPS))
    nmse_db = 10.0 * math.log10(s_err_pow / s_true_pow)
    print(f"[Stage 1] 3.0s Auto-Calibration Complete:")
    print(f"          Secondary Path S_hat(z) Modeling Accuracy (NMSE): {nmse_db:.1f} dB")

    # ------------------------------------------------------------------------
    # STAGE 2: Real-Time Traffic Rumble Cancellation (10 Seconds @ 4,000 Hz)
    # ------------------------------------------------------------------------
    duration_sec = 10.0
    total_samples = int(duration_sec * FS)
    outdoor_source = generate_traffic_rumble(total_samples)

    # Delay lines for true acoustic simulation and DSP controller
    src_hist = [0.0] * W_TAPS       # True outdoor wave history
    spk_hist = [0.0] * S_TAPS       # True speaker output history
    x_buf = [0.0] * W_TAPS          # Controller clean reference buffer
    s_buf = [0.0] * S_TAPS          # Controller secondary path buffer
    xf_buf = [0.0] * W_TAPS         # Controller filtered-x buffer
    y_buf = [0.0] * S_TAPS          # Controller speaker history for feedback cancellation

    room_anc_off = [0.0] * total_samples
    room_anc_on = [0.0] * total_samples

    print(f"\n[Stage 2] Running Real-Time Sample-by-Sample FxLMS ({duration_sec:.0f}s @ {FS} Hz)...")
    print("-" * 70)
    print(f"{'Time Window':<16} | {'ANC OFF (RMS)':<15} | {'ANC ON (RMS)':<15} | {'Attenuation':<15}")
    print("-" * 70)

    for n in range(total_samples):
        # 1. Outdoor wave arrives at Reference Mic
        u_n = outdoor_source[n]
        src_hist.insert(0, u_n)
        src_hist.pop()

        # True Primary Path sound arriving at Indoor Error Mic (without ANC)
        d_n = dot_product(P_true, src_hist, W_TAPS)
        room_anc_off[n] = d_n

        # True Reference Mic reading = Outdoor wave + Acoustic Feedback from Speaker
        x_mic = u_n + dot_product(F_true, spk_hist, S_TAPS)

        # 2a. DSP Feedback Neutralization
        fb_est = dot_product(F_hat, y_buf, S_TAPS)
        x_clean = x_mic - fb_est

        # 2b. Push clean reference into control filter delay line
        x_buf.insert(0, x_clean)
        x_buf.pop()

        # 2c. Compute anti-noise control signal y(n) = W^T * x_buf
        y_n = dot_product(W, x_buf, W_TAPS)
        speaker_out = max(-0.95, min(0.95, -y_n))

        # 2d. Drive speaker & compute true physical acoustic superposition at Error Mic
        spk_hist.insert(0, speaker_out)
        spk_hist.pop()
        anti_noise_at_err_mic = dot_product(S_true, spk_hist, S_TAPS)

        # Residual error at Indoor Error Mic: e(n) = Primary Noise + Speaker Anti-Noise
        e_n = d_n + anti_noise_at_err_mic
        room_anc_on[n] = e_n

        # 2e. Update DSP filtered-reference x'(n) = S_hat^T * s_buf
        y_buf.insert(0, speaker_out)
        y_buf.pop()
        s_buf.insert(0, x_clean)
        s_buf.pop()

        xf_n = dot_product(S_hat, s_buf, S_TAPS)
        xf_buf.insert(0, xf_n)
        xf_buf.pop()

        # 2f. Normalized Leaky FxLMS Weight Update
        xf_power = dot_product(xf_buf, xf_buf, W_TAPS) + EPSILON
        norm_step = (MU_FXLMS * e_n) / xf_power
        for k in range(W_TAPS):
            W[k] = (LEAKAGE * W[k]) + norm_step * xf_buf[k]

        # Print progress every 2 seconds
        if (n + 1) % (2 * FS) == 0:
            win_start = n + 1 - (2 * FS)
            rms_off = math.sqrt(sum(x * x for x in room_anc_off[win_start:n + 1]) / (2 * FS))
            rms_on = math.sqrt(sum(x * x for x in room_anc_on[win_start:n + 1]) / (2 * FS))
            atten_db = 20.0 * math.log10((rms_on + 1e-9) / (rms_off + 1e-9))
            t_end = (n + 1) // FS
            print(f"  {t_end - 2:>2d}.0s - {t_end:>2d}.0s   | {rms_off:<15.4f} | {rms_on:<15.4f} | {atten_db:>8.1f} dB")

    # Steady-state summary (last 4 seconds)
    ss_start = int(6.0 * FS)
    ss_off = math.sqrt(sum(x * x for x in room_anc_off[ss_start:]) / (total_samples - ss_start))
    ss_on = math.sqrt(sum(x * x for x in room_anc_on[ss_start:]) / (total_samples - ss_start))
    ss_db = 20.0 * math.log10(ss_on / ss_off)
    energy_removed_pct = (1.0 - (ss_on / ss_off) ** 2) * 100.0

    print("-" * 70)
    print(f"STEADY-STATE REDUCTION IN THIS TOY MODEL : {ss_db:.1f} dB ({energy_removed_pct:.1f}% of the modelled energy)")
    print("(ideal electronics, hand-made paths; see the docstring before quoting this number)")
    print("=" * 70)

    # Save WAV files for listening comparison
    out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
    wav_off = os.path.join(out_dir, "1_room_anc_off.wav")
    wav_on = os.path.join(out_dir, "2_room_anc_on.wav")
    wav_demo = os.path.join(out_dir, "3_demo_off_then_on.wav")

    # Build a 10-second split demo: first 4 seconds ANC OFF, next 6 seconds steady-state ANC ON
    demo_split = room_anc_off[: int(4.0 * FS)] + room_anc_on[int(4.0 * FS) :]

    save_wav(wav_off, room_anc_off)
    save_wav(wav_on, room_anc_on)
    save_wav(wav_demo, demo_split)

    print("\nSaved playable 16-bit WAV audio files:")
    print(f"  1. {wav_off}")
    print(f"  2. {wav_on}")
    print(f"  3. {wav_demo}  <-- Listen to this! (0-4s ANC OFF, 4-10s ANC ON)")


if __name__ == "__main__":
    run_simulation()
