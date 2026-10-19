#!/usr/bin/env python3
"""
Real-World YouTube Traffic Audio ANC Simulator
==============================================
Video: "India Traffic Jam Sound || Heavy Traffic Noise" (https://youtu.be/qNK0MmYodLg)

Analyzes the acoustic spectrum of the downloaded YouTube recording and simulates:
1. Bare Open Window (Uncancelled street noise entering the room)
2. Active FxLMS Woofer Only (Cancels <300 Hz bass, but >400 Hz horns pass through)
3. Passive Foam-Lined Window Baffle Only (Muffles >350 Hz horns, but <300 Hz bass passes through)
4. Full Hybrid System: Passive Window Baffle + Active FxLMS Woofer (Eliminates both!)
"""

import math
import os
import random
import subprocess
import wave
from array import array

FS_FULL = 16000                 # Full-band audio rate (16 kHz) for realistic listening
DECIM = 4                       # 4x decimation -> 4,000 Hz ANC controller rate
FS_ANC = FS_FULL // DECIM       # 4,000 Hz (matches Teensy 4.0 ISR rate)

W_TAPS = 128                    # Control filter length @ 4 kHz
S_TAPS = 64                     # Secondary path length @ 4 kHz
MU_S_IDENT = 0.08
MU_FXLMS = 0.018
LEAKAGE = 0.99995
EPSILON = 1e-4


def dot_product(a, b, n):
    s = 0.0
    for i in range(n):
        s += a[i] * b[i]
    return s


def load_wav_mono(path, target_fs):
    """Convert audio file to mono 16-bit WAV at target_fs via ffmpeg and load samples."""
    tmp_wav = path + f".{target_fs}hz.tmp.wav"
    subprocess.run(
        [
            "ffmpeg", "-y", "-i", path,
            "-ac", "1", "-ar", str(target_fs),
            "-sample_fmt", "s16", tmp_wav,
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=True,
    )
    with wave.open(tmp_wav, "rb") as wf:
        raw = wf.readframes(wf.getnframes())
    os.remove(tmp_wav)
    pcm = array("h")
    pcm.frombytes(raw)
    # Normalize peak to ~0.85
    peak = max(abs(x) for x in pcm) or 1
    scale = 0.85 / float(peak)
    return [x * scale for x in pcm]


def save_wav(filename, samples, fs=FS_FULL):
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


class BiquadFilter:
    """2nd-order Butterworth IIR filter (Low-Pass, High-Pass, or Band-Pass)."""

    def __init__(self, b0, b1, b2, a1, a2):
        self.b0, self.b1, self.b2 = b0, b1, b2
        self.a1, self.a2 = a1, a2
        self.x1 = self.x2 = 0.0
        self.y1 = self.y2 = 0.0

    @staticmethod
    def lowpass(fc, fs, q=0.7071):
        w0 = 2.0 * math.pi * fc / fs
        alpha = math.sin(w0) / (2.0 * q)
        cos_w0 = math.cos(w0)
        b0 = (1.0 - cos_w0) / 2.0
        b1 = 1.0 - cos_w0
        b2 = (1.0 - cos_w0) / 2.0
        a0 = 1.0 + alpha
        a1 = -2.0 * cos_w0
        a2 = 1.0 - alpha
        return BiquadFilter(b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)

    @staticmethod
    def highpass(fc, fs, q=0.7071):
        w0 = 2.0 * math.pi * fc / fs
        alpha = math.sin(w0) / (2.0 * q)
        cos_w0 = math.cos(w0)
        b0 = (1.0 + cos_w0) / 2.0
        b1 = -(1.0 + cos_w0)
        b2 = (1.0 + cos_w0) / 2.0
        a0 = 1.0 + alpha
        a1 = -2.0 * cos_w0
        a2 = 1.0 - alpha
        return BiquadFilter(b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0)

    def process(self, x):
        y = self.b0 * x + self.b1 * self.x1 + self.b2 * self.x2 - self.a1 * self.y1 - self.a2 * self.y2
        self.x2, self.x1 = self.x1, x
        self.y2, self.y1 = self.y1, y
        return y

    def process_array(self, arr):
        return [self.process(x) for x in arr]


def rms(arr):
    if not arr:
        return 0.0
    return math.sqrt(sum(x * x for x in arr) / len(arr))


def db_ratio(val_rms, ref_rms):
    return 20.0 * math.log10((val_rms + 1e-9) / (ref_rms + 1e-9))


def build_acoustic_paths():
    """4,000 Hz discrete-time impulse responses for the window geometry."""
    P_true = [0.0] * W_TAPS
    S_true = [0.0] * S_TAPS
    F_true = [0.0] * S_TAPS

    # Primary Path P(z): 7 samples (60 cm) delay + window aperture dispersion
    P_true[7] = 0.82
    P_true[8] = 0.45
    P_true[9] = -0.18
    P_true[10] = -0.12
    for k in range(11, 55):
        t = k - 7
        P_true[k] = 0.22 * math.exp(-0.11 * t) * math.cos(0.42 * t)

    # Secondary Path S(z): 3 samples (25 cm) delay + 5.25" sealed woofer phase response
    S_true[3] = 0.75
    S_true[4] = 0.52
    S_true[5] = -0.28
    S_true[6] = -0.19
    for k in range(7, 36):
        t = k - 3
        S_true[k] = 0.18 * math.exp(-0.16 * t) * math.sin(0.55 * t)

    # Feedback Path F(z): 4 samples (35 cm) delay, cardioid reference mic rejection
    F_true[4] = 0.16
    F_true[5] = 0.09
    F_true[6] = -0.06
    for k in range(7, 25):
        t = k - 4
        F_true[k] = 0.04 * math.exp(-0.20 * t) * math.cos(0.50 * t)

    return P_true, S_true, F_true


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    yt_wav = os.path.join(base_dir, "input_youtube.wav")
    out_dir = os.path.join(base_dir, "output_youtube")

    print("=" * 74)
    print(" REAL YOUTUBE AUDIO TEST: 'India Traffic Jam Sound || Heavy Traffic Noise'")
    print("=" * 74)

    # Load 20 seconds of full-band 16 kHz audio
    raw_16k = load_wav_mono(yt_wav, FS_FULL)[: 20 * FS_FULL]
    N_16k = len(raw_16k)

    # ------------------------------------------------------------------------
    # 1. SPECTRUM BREAKDOWN OF THE YOUTUBE TRAFFIC RECORDING
    # ------------------------------------------------------------------------
    # Split into 3 acoustic bands to inspect what's inside this recording:
    # - Bass / Engine Rumble: 30 - 300 Hz (Target of Active FxLMS Woofer)
    # - Midrange Horns / Engines: 300 - 1,200 Hz (Blocked by Passive Foam Baffle)
    # - High-Freq Honks / Hiss: 1,200 - 8,000 Hz (Blocked by Passive Foam Baffle)
    bass_band = BiquadFilter.lowpass(300.0, FS_FULL).process_array(
        BiquadFilter.highpass(30.0, FS_FULL).process_array(raw_16k)
    )
    mid_band = BiquadFilter.lowpass(1200.0, FS_FULL).process_array(
        BiquadFilter.highpass(300.0, FS_FULL).process_array(raw_16k)
    )
    high_band = BiquadFilter.highpass(1200.0, FS_FULL).process_array(raw_16k)

    total_e = sum(x * x for x in raw_16k)
    bass_pct = 100.0 * sum(x * x for x in bass_band) / total_e
    mid_pct = 100.0 * sum(x * x for x in mid_band) / total_e
    high_pct = 100.0 * sum(x * x for x in high_band) / total_e

    print("\n[1] FREQUENCY SPECTRUM ANALYSIS OF YOUR YOUTUBE CLIP:")
    print(f"    - Low-Freq Bass & Engine Rumble (30 - 300 Hz)   : {bass_pct:5.1f}% of acoustic energy")
    print(f"    - Mid-Range Vehicle Horns       (300 - 1200 Hz) : {mid_pct:5.1f}% of acoustic energy")
    print(f"    - High-Pitched Honking & Hiss   (1200 - 8000 Hz): {high_pct:5.1f}% of acoustic energy")
    print("    -> Notice how Indian traffic is dominated by mid/high-pitched horns (>300 Hz)!")
    print("       Let's see how Active ANC alone vs. Hybrid Baffle + Active ANC handles this.")

    # ------------------------------------------------------------------------
    # 2. RUN 4,000 Hz FxLMS CONTROLLER ON THE LOW-FREQUENCY PATH (< 300 Hz)
    # ------------------------------------------------------------------------
    P_true, S_true, F_true = build_acoustic_paths()
    W = [0.0] * W_TAPS
    S_hat = [0.0] * S_TAPS
    F_hat = [0.0] * S_TAPS

    # Stage 1: 3-second Secondary Path Calibration
    calib_samples = 3 * FS_ANC
    y_hist = [0.0] * S_TAPS
    rng = random.Random(99)
    for _ in range(calib_samples):
        v_n = rng.uniform(-0.18, 0.18)
        y_hist.insert(0, v_n)
        y_hist.pop()
        e_n = dot_product(S_true, y_hist, S_TAPS) + rng.gauss(0.0, 0.001)
        x_n = dot_product(F_true, y_hist, S_TAPS) + rng.gauss(0.0, 0.001)
        e_hat = dot_product(S_hat, y_hist, S_TAPS)
        f_hat = dot_product(F_hat, y_hist, S_TAPS)
        p_v = dot_product(y_hist, y_hist, S_TAPS) + EPSILON
        for k in range(S_TAPS):
            S_hat[k] += (MU_S_IDENT * (e_n - e_hat) / p_v) * y_hist[k]
            F_hat[k] += (MU_S_IDENT * (x_n - f_hat) / p_v) * y_hist[k]

    # Downsample low-pass filtered outdoor mic signal (< 320 Hz anti-aliasing) to 4 kHz
    aa_filter = BiquadFilter.lowpass(320.0, FS_FULL)
    hp_filter = BiquadFilter.highpass(25.0, FS_FULL)
    outdoor_lp_16k = hp_filter.process_array(aa_filter.process_array(raw_16k))
    outdoor_4k = outdoor_lp_16k[::DECIM]
    N_4k = len(outdoor_4k)

    src_hist = [0.0] * W_TAPS
    spk_hist = [0.0] * S_TAPS
    x_buf = [0.0] * W_TAPS
    s_buf = [0.0] * S_TAPS
    xf_buf = [0.0] * W_TAPS
    y_buf = [0.0] * S_TAPS

    bass_off_4k = [0.0] * N_4k
    bass_on_4k = [0.0] * N_4k

    for n in range(N_4k):
        u_n = outdoor_4k[n]
        src_hist.insert(0, u_n)
        src_hist.pop()

        d_n = dot_product(P_true, src_hist, W_TAPS)
        bass_off_4k[n] = d_n

        x_mic = u_n + dot_product(F_true, spk_hist, S_TAPS)
        x_clean = x_mic - dot_product(F_hat, y_buf, S_TAPS)

        x_buf.insert(0, x_clean)
        x_buf.pop()

        y_n = dot_product(W, x_buf, W_TAPS)
        speaker_out = max(-0.95, min(0.95, -y_n))

        spk_hist.insert(0, speaker_out)
        spk_hist.pop()
        e_n = d_n + dot_product(S_true, spk_hist, S_TAPS)
        bass_on_4k[n] = e_n

        y_buf.insert(0, speaker_out)
        y_buf.pop()
        s_buf.insert(0, x_clean)
        s_buf.pop()

        xf_n = dot_product(S_hat, s_buf, S_TAPS)
        xf_buf.insert(0, xf_n)
        xf_buf.pop()

        xf_power = dot_product(xf_buf, xf_buf, W_TAPS) + EPSILON
        norm_step = (MU_FXLMS * e_n) / xf_power
        for k in range(W_TAPS):
            W[k] = (LEAKAGE * W[k]) + norm_step * xf_buf[k]

    # Upsample 4 kHz bass signals back to 16 kHz with reconstruction filter
    def upsample_to_16k(sig_4k):
        up = [0.0] * (len(sig_4k) * DECIM)
        for i, val in enumerate(sig_4k):
            for m in range(DECIM):
                up[i * DECIM + m] = val
        rec1 = BiquadFilter.lowpass(340.0, FS_FULL)
        rec2 = BiquadFilter.lowpass(340.0, FS_FULL)
        return rec2.process_array(rec1.process_array(up))

    bass_off_16k = upsample_to_16k(bass_off_4k)
    bass_on_16k = upsample_to_16k(bass_on_4k)

    # High/Mid frequencies (> 320 Hz) that bypass the 320 Hz ANC woofer band:
    # Delay by 28 samples at 16 kHz (7 samples @ 4 kHz = 1.75 ms acoustic air delay)
    high_mid_16k = BiquadFilter.highpass(320.0, FS_FULL).process_array(raw_16k)
    high_mid_delayed = [0.0] * 28 + high_mid_16k[:-28]

    # Model the Passive Foam-Lined S-Curve Window Baffle on frequencies > 320 Hz:
    # Acoustic foam + two 90-degree bends provide progressive high-frequency absorption
    # (~16-22 dB reduction above 400 Hz)
    baffle_lp1 = BiquadFilter.lowpass(380.0, FS_FULL)
    baffle_lp2 = BiquadFilter.lowpass(650.0, FS_FULL)
    high_mid_baffled = [
        0.14 * x for x in baffle_lp2.process_array(baffle_lp1.process_array(high_mid_delayed))
    ]

    # Construct the 4 full-band 16 kHz room signals:
    # 1. Bare Open Window (Bass OFF + Unbaffled Highs/Horns)
    sig_bare_window = [bass_off_16k[i] + high_mid_delayed[i] for i in range(N_16k)]

    # 2. Active FxLMS Only on Bare Window (Bass Cancelled + Unbaffled Highs/Horns)
    sig_anc_only = [bass_on_16k[i] + high_mid_delayed[i] for i in range(N_16k)]

    # 3. Passive Foam Baffle Only, ANC OFF (Uncancelled Bass + Baffled Highs/Horns)
    sig_baffle_only = [bass_off_16k[i] + high_mid_baffled[i] for i in range(N_16k)]

    # 4. Full Hybrid System: Passive Foam Baffle + Active FxLMS Woofer ON
    sig_hybrid_full = [bass_on_16k[i] + high_mid_baffled[i] for i in range(N_16k)]

    # Evaluate Steady-State Metrics (from t = 4s to 20s)
    s_idx = 4 * FS_FULL
    rms_bass_off = rms(bass_off_16k[s_idx:])
    rms_bass_on = rms(bass_on_16k[s_idx:])
    bass_atten_db = db_ratio(rms_bass_on, rms_bass_off)

    rms_bare = rms(sig_bare_window[s_idx:])
    rms_anc = rms(sig_anc_only[s_idx:])
    rms_baffle = rms(sig_baffle_only[s_idx:])
    rms_hybrid = rms(sig_hybrid_full[s_idx:])

    print("\n[2] SIMULATION RESULTS ON YOUR YOUTUBE TRAFFIC AUDIO (t = 4s to 20s):")
    print("-" * 74)
    print(f"  Low-Frequency Bass Band (<320 Hz) FxLMS Cancellation : {bass_atten_db:>6.1f} dB")
    print("-" * 74)
    print(f"  Scenario A: Bare Open Window (No Baffle, No ANC)     :   0.0 dB (Baseline)")
    print(f"  Scenario B: Active FxLMS Woofer Only (No Baffle)     : {db_ratio(rms_anc, rms_bare):>6.1f} dB total")
    print(f"  Scenario C: Passive Foam Baffle Only (ANC OFF)       : {db_ratio(rms_baffle, rms_bare):>6.1f} dB total")
    print(f"  Scenario D: HYBRID (Foam Baffle + Active FxLMS ON)   : {db_ratio(rms_hybrid, rms_bare):>6.1f} dB total")
    print("-" * 74)

    # Build a 20-second 4-Stage Comparison WAV (5 seconds per stage):
    # 0-5s  : Bare Open Window
    # 5-10s : Active ANC Only (No Baffle)
    # 10-15s: Passive Baffle Only (ANC Off)
    # 15-20s: Full Hybrid (Passive Baffle + Active FxLMS ON)
    s5 = 5 * FS_FULL
    demo_4_stages = (
        sig_bare_window[0:s5]
        + sig_anc_only[s5 : 2 * s5]
        + sig_baffle_only[2 * s5 : 3 * s5]
        + sig_hybrid_full[3 * s5 : 4 * s5]
    )

    # Also build a Bass-Only comparison (4s Bass ANC OFF -> 12s Bass ANC ON)
    demo_bass_only = bass_off_16k[: 4 * FS_FULL] + bass_on_16k[4 * FS_FULL : 16 * FS_FULL]

    save_wav(os.path.join(out_dir, "1_bare_open_window.wav"), sig_bare_window)
    save_wav(os.path.join(out_dir, "2_active_anc_only_no_baffle.wav"), sig_anc_only)
    save_wav(os.path.join(out_dir, "3_passive_baffle_only_anc_off.wav"), sig_baffle_only)
    save_wav(os.path.join(out_dir, "4_hybrid_baffle_plus_active_anc.wav"), sig_hybrid_full)
    save_wav(os.path.join(out_dir, "5_DEMO_4_STAGES_COMPARISON.wav"), demo_4_stages)
    save_wav(os.path.join(out_dir, "6_DEMO_BASS_BAND_OFF_THEN_ON.wav"), demo_bass_only)

    print("\n[3] GENERATED PLAYABLE WAV FILES:")
    print(f"  - {os.path.join(out_dir, '5_DEMO_4_STAGES_COMPARISON.wav')}")
    print(f"  - {os.path.join(out_dir, '6_DEMO_BASS_BAND_OFF_THEN_ON.wav')}")
    print("=" * 74)


if __name__ == "__main__":
    main()
