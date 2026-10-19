#!/usr/bin/env python3
"""
PHASE & TIMING CALCULATOR FOR FEED-FORWARD ANTI-NOISE
=====================================================
Authors: viaanshetty3-debug & lishirao-star

What this program does:
1. Loads an incoming sound x(t) (a WAV file, a live mic capture, or a test tone set).
2. Estimates dominant tones: Blackman-Harris FFT + log-parabolic peak interpolation,
   then a least-squares fit of amplitude and phase for every detected tone.
3. Computes the timing budget for a feed-forward canceller:
       t_wait = t_acoustic - t_compute - t_hardware
   and builds the inverted, delayed anti-noise (-1 x, sub-sample windowed-sinc delay).
4. Reports what that anti-noise would leave behind when the speaker path is NOT
   perfect (gain error and timing error you can set), using the actual signals.

IMPORTANT - what this does NOT prove:
  * With perfect gain and timing the residual is exactly zero BY CONSTRUCTION
    (it is the identity x - x = 0). That is arithmetic, not evidence of anything.
  * This script processes a recording that is already fully known. A real
    controller cannot do that: it only has samples the reference mic has already
    heard, so broadband noise is cancelled only as far as the system is causal
    (see simulator/mismatch_sweep.py).
  * It models one point (the speaker plane). Silence there says nothing about the
    listener's ears; the quiet zone around the cancellation point is roughly
    lambda/10 in size, i.e. centimetres at 300 Hz.
"""

import math
import os
import subprocess
import sys
import time
import wave
import numpy as np
from scipy import signal

# ============================================================================
# 1. PHYSICAL & HARDWARE CONSTANTS
# ============================================================================
FS = 48000                          # 48,000 Hz precision sample rate (1 sample = 20.833 us)
TEMP_CELSIUS = 22.0                 # Room/outdoor air temperature (C)
# Speed of sound in dry air, ideal gas (same formula as tools/timing_model.py); humidity ignored.
C_SOUND = math.sqrt(1.4 * 8.314462618 * (TEMP_CELSIUS + 273.15) / 0.0289647)   # 344.4 m/s at 22 C

DEFAULT_MIC_TO_SPK_M = 0.350        # 35.0 cm (0.350 m) from Outdoor Mic to Speaker
DEFAULT_HW_DAC_DELAY_US = 12.5      # Hardware DAC + Analog Amp propagation delay (us)
                                    # NOTE: excludes analog anti-alias/reconstruction filters, which
                                    # add ~0.5-1.5 ms in the RC design (see HARDWARE_AND_BAFFLE_GUIDE.md)
DEFAULT_GAIN_ERROR = 0.05           # assumed 5% speaker/amp/mic gain error (adjustable)
DEFAULT_DELAY_ERROR_US = 50.0       # assumed 50 us timing error (adjustable)


# ============================================================================
# 2. SUB-SAMPLE FRACTIONAL DELAY FILTER (Windowed Sinc Interpolation)
# ============================================================================
def apply_exact_delay_and_invert(waveform, total_delay_samples):
    """
    Delays `waveform` by `total_delay_samples` (integer + fractional part)
    using a 64-tap Blackman-windowed Sinc filter and inverts phase by exactly
    180.0000 degrees (multiplies by -1.0) for zero-error destructive interference.
    """
    int_delay = int(np.floor(total_delay_samples))
    frac_delay = total_delay_samples - int_delay

    # If fractional delay is negligible, perform exact integer shift + inversion
    if abs(frac_delay) < 1e-9:
        out = np.zeros_like(waveform, dtype=np.float64)
        if int_delay == 0:
            return -1.0 * waveform
        elif int_delay > 0 and int_delay < len(waveform):
            out[int_delay:] = -1.0 * waveform[:-int_delay]
        return out

    # 64-tap windowed-Sinc filter for sub-microsecond fractional sample accuracy
    half_win = 32
    taps = np.arange(-half_win, half_win + 1, dtype=np.float64)
    sinc_kernel = np.sinc(taps - frac_delay) * np.blackman(2 * half_win + 1)
    sinc_kernel /= np.sum(sinc_kernel)

    # Filter and shift by integer delay
    filtered = signal.convolve(waveform, sinc_kernel, mode="same")
    out = np.zeros_like(waveform, dtype=np.float64)
    if int_delay == 0:
        out = -1.0 * filtered
    elif 0 < int_delay < len(waveform):
        out[int_delay:] = -1.0 * filtered[:-int_delay]
    return out


# ============================================================================
# 3. HIGH-PRECISION FREQUENCY, PHASE & HARMONIC DETECTOR
# ============================================================================
def analyze_sound_details(waveform, fs=FS, num_harmonics=6):
    """
    Extracts exact frequencies (Hz), amplitudes, phases (deg/rad), wavelengths (cm),
    and periods (ms) of the incoming sound using Blackman-Harris FFT + parabolic
    bin interpolation + exact Least-Squares phase/amplitude fitting.
    """
    t0_ns = time.perf_counter_ns()

    N = min(len(waveform), fs * 2)  # Analyze up to 2 seconds for ultra-fine frequency bins
    seg = np.asarray(waveform[:N], dtype=np.float64)
    seg_dc = seg - np.mean(seg)

    # RMS & Peak metrics
    rms_val = float(np.sqrt(np.mean(seg_dc ** 2)))
    peak_val = float(np.max(np.abs(seg_dc)))
    db_fs = 20.0 * math.log10(rms_val + 1e-12)

    # Windowed FFT with parabolic peak interpolation
    win = signal.windows.blackmanharris(N)
    spec = np.fft.rfft(seg_dc * win)
    mag = np.abs(spec)
    freqs = np.fft.rfftfreq(N, d=1.0 / fs)

    # Ignore DC (< 20 Hz)
    min_bin = max(1, int(20.0 * N / fs))
    mag[:min_bin] = 0.0

    # Find top spectral peaks separated by at least 12 Hz
    min_sep_bins = max(2, int(12.0 * N / fs))
    mag_copy = mag.copy()
    detected_freqs = []

    for _ in range(num_harmonics):
        pk_bin = int(np.argmax(mag_copy))
        if mag_copy[pk_bin] < 1e-6:
            break
        # Parabolic interpolation around peak bin for sub-Hz precision
        if 1 <= pk_bin < len(mag) - 1:
            alpha = math.log(mag[pk_bin - 1] + 1e-15)
            beta = math.log(mag[pk_bin] + 1e-15)
            gamma = math.log(mag[pk_bin + 1] + 1e-15)
            denom = alpha - 2.0 * beta + gamma
            delta = 0.5 * (alpha - gamma) / denom if abs(denom) > 1e-12 else 0.0
        else:
            delta = 0.0

        exact_freq = (pk_bin + delta) * (fs / float(N))
        detected_freqs.append(exact_freq)

        # Suppress neighborhood to find next distinct tone/harmonic
        lo = max(0, pk_bin - min_sep_bins)
        hi = min(len(mag_copy), pk_bin + min_sep_bins + 1)
        mag_copy[lo:hi] = 0.0

    detected_freqs.sort()

    # Exact Least-Squares Fit for Amplitude & Phase of each frequency component:
    # x(t) ≈ sum_k [ a_k cos(2*pi*f_k*t) + b_k sin(2*pi*f_k*t) ]
    t_vec = np.arange(N, dtype=np.float64) / float(fs)
    K = len(detected_freqs)
    harmonics_info = []

    if K > 0:
        design_cols = []
        for f_k in detected_freqs:
            w_t = 2.0 * np.pi * f_k * t_vec
            design_cols.append(np.cos(w_t))
            design_cols.append(np.sin(w_t))
        A_mat = np.column_stack(design_cols)
        coeffs, _, _, _ = np.linalg.lstsq(A_mat, seg_dc, rcond=None)

        for idx, f_k in enumerate(detected_freqs):
            a_k = coeffs[2 * idx]
            b_k = coeffs[2 * idx + 1]
            # a_k cos(wt) + b_k sin(wt) = Amp * cos(wt + phi), where phi = atan2(-b_k, a_k)
            amp_k = float(math.hypot(a_k, b_k))
            phi_rad = float(math.atan2(-b_k, a_k))
            phi_deg = math.degrees(phi_rad)
            opp_phi_deg = ((phi_deg + 180.0 + 180.0) % 360.0) - 180.0

            period_ms = 1000.0 / f_k
            wavelength_m = C_SOUND / f_k

            harmonics_info.append({
                "index": idx + 1,
                "freq_hz": round(f_k, 4),
                "period_ms": round(period_ms, 4),
                "half_period_180_us": round((period_ms * 1000.0) / 2.0, 2),
                "wavelength_cm": round(wavelength_m * 100.0, 2),
                "amplitude": round(amp_k, 5),
                "phase_deg": round(phi_deg, 3),
                "opposite_phase_deg": round(opp_phi_deg, 3),
                "phase_rad": phi_rad,
            })

    compute_time_us = (time.perf_counter_ns() - t0_ns) / 1000.0
    dominant = max(harmonics_info, key=lambda h: h["amplitude"]) if harmonics_info else None

    return {
        "sample_rate_hz": fs,
        "num_samples": len(waveform),
        "duration_sec": round(len(waveform) / float(fs), 3),
        "rms_level": round(rms_val, 6),
        "peak_level": round(peak_val, 6),
        "level_dbfs": round(db_fs, 2),
        "speed_of_sound_m_s": round(C_SOUND, 3),
        "dominant_tone": dominant,
        "components": harmonics_info,
        "analysis_compute_time_us": round(compute_time_us, 2),
    }


# ============================================================================
# 4. ZERO-ERROR TIMING CALCULATOR & OPPOSITE-PHASE WAVE GENERATOR
# ============================================================================
def compute_zero_error_cancellation(
    waveform,
    fs=FS,
    mic_to_speaker_m=DEFAULT_MIC_TO_SPK_M,
    hw_dac_delay_us=DEFAULT_HW_DAC_DELAY_US,
    gain_error=DEFAULT_GAIN_ERROR,
    delay_error_us=DEFAULT_DELAY_ERROR_US,
):
    """
    1. Analyzes the incoming sound wave.
    2. Measures the per-sample Python/DSP inversion time.
    3. Computes the exact time required for sound to travel from Mic -> Speaker
       and subtracts computation + hardware delay to determine the exact digital
       hold time `t_wait_us`.
    4. Generates the opposite-phase speaker output (`speaker_anti_wave`).
       `ideal_identity_rms` is the residual with a perfect speaker path; it is zero
       by construction and is NOT a validation. `residual_error_rms` is the residual
       when the speaker path has `gain_error` (fraction, e.g. 0.05 = 5% too quiet)
       and `delay_error_us` extra delay, which is what real hardware looks like.
    """
    details = analyze_sound_details(waveform, fs=fs)

    # Time of one numpy negation (NOT a real-time DSP measurement; kept only as a lower bound)
    t_start_ns = time.perf_counter_ns()
    _ = -1.0 * waveform[:1024]
    per_sample_dsp_us = ((time.perf_counter_ns() - t_start_ns) / 1000.0) / 1024.0

    # Acoustic travel time from Outdoor Mic to Speaker (in microseconds)
    t_acoustic_us = (mic_to_speaker_m / C_SOUND) * 1e6
    t_total_elec_us = per_sample_dsp_us + hw_dac_delay_us

    # Exact digital wait time required so the speaker fires at the exact microsecond
    # the incoming acoustic wavefront reaches the speaker cone:
    t_wait_us = t_acoustic_us - t_total_elec_us
    if t_wait_us < 0:
        raise ValueError(
            f"Causality violation! Mic is too close ({mic_to_speaker_m*100:.1f} cm -> {t_acoustic_us:.1f} us) "
            f"for hardware delay ({t_total_elec_us:.1f} us). Move mic further outside!"
        )

    exact_acoustic_samples = (t_acoustic_us * 1e-6) * fs
    exact_wait_samples = (t_wait_us * 1e-6) * fs
    exact_elec_samples = (t_total_elec_us * 1e-6) * fs

    # 1. What arrives at the speaker plane naturally through the air after `t_acoustic_us`:
    # (Delay by exact_acoustic_samples, WITHOUT phase inversion)
    incoming_at_speaker = -1.0 * apply_exact_delay_and_invert(waveform, exact_acoustic_samples)

    # 2. What our Python program sends to the DAC after holding for `t_wait_us`,
    # which then passes through `t_total_elec_us` hardware delay to reach the speaker cone
    # at `exact_wait_samples + exact_elec_samples == exact_acoustic_samples`:
    speaker_anti_wave = apply_exact_delay_and_invert(
        waveform, exact_wait_samples + exact_elec_samples
    )

    # 3. Superposition at the speaker plane.
    warmup = int(math.ceil(exact_acoustic_samples)) + 34
    ideal_residual = incoming_at_speaker + speaker_anti_wave
    ideal_rms = float(np.sqrt(np.mean(ideal_residual[warmup:] ** 2)))

    # Realistic speaker path: gain error and extra delay (computed from the real signals)
    extra_samples = delay_error_us * 1e-6 * fs
    speaker_actual = (1.0 - gain_error) * apply_exact_delay_and_invert(
        waveform, exact_wait_samples + exact_elec_samples + extra_samples
    )
    residual_wave = incoming_at_speaker + speaker_actual

    rms_in = float(np.sqrt(np.mean(incoming_at_speaker[warmup:] ** 2)))
    rms_res = float(np.sqrt(np.mean(residual_wave[warmup:] ** 2)))
    atten_db = 20.0 * math.log10((rms_res + 1e-18) / (rms_in + 1e-18))

    timing_budget = {
        "mic_to_speaker_distance_cm": round(mic_to_speaker_m * 100.0, 2),
        "speed_of_sound_m_s": round(C_SOUND, 3),
        "1_acoustic_travel_time_us": round(t_acoustic_us, 3),
        "2_python_dsp_per_sample_us": round(per_sample_dsp_us, 4),
        "3_hardware_dac_amp_delay_us": round(hw_dac_delay_us, 3),
        "4_exact_digital_wait_time_us": round(t_wait_us, 3),
        "exact_wait_samples_at_48khz": round(exact_wait_samples, 4),
        "phase_inversion_applied_deg": 180.0,
        "polarity_multiplier": -1.0,
        "incoming_rms_at_speaker": round(rms_in, 8),
        "assumed_gain_error": gain_error,
        "assumed_delay_error_us": delay_error_us,
        "ideal_identity_rms": ideal_rms,
        "residual_error_rms": round(rms_res, 8),
        "cancellation_percentage": round((1.0 - rms_res / (rms_in + 1e-18)) * 100.0, 3),
        "attenuation_db": round(atten_db, 2),
    }

    return details, timing_budget, incoming_at_speaker, speaker_anti_wave, residual_wave


# ============================================================================
# 5. SAVE & PLAY OPPOSITE-PHASE AUDIO ON TABLET / TERMINAL
# ============================================================================
def save_wav_mono(path, samples, fs=FS):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    pcm16 = np.clip(samples * 32767.0, -32767, 32767).astype(np.int16)
    with wave.open(path, "wb") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(fs)
        wf.writeframes(pcm16.tobytes())


def save_wav_stereo(path, left, right, fs=FS):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    stereo = np.column_stack((left, right))
    pcm16 = np.clip(stereo * 32767.0, -32767, 32767).astype(np.int16)
    with wave.open(path, "wb") as wf:
        wf.setnchannels(2)
        wf.setsampwidth(2)
        wf.setframerate(fs)
        wf.writeframes(pcm16.tobytes())


def capture_live_microphone_wav(duration_sec=3, fs=FS):
    """
    Captures live audio from the Android tablet microphone using `termux-microphone-record`
    and converts it to a 48,000 Hz 16-bit mono WAV for precision analysis.
    """
    base_dir = os.path.dirname(os.path.abspath(__file__))
    raw_rec = os.path.join(base_dir, "output", "live_mic_raw.m4a")
    wav_rec = os.path.join(base_dir, "output", "live_mic_captured.wav")
    os.makedirs(os.path.dirname(raw_rec), exist_ok=True)

    # Stop any ongoing recording and remove old temp file
    subprocess.run(["termux-microphone-record", "-q"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if os.path.exists(raw_rec):
        os.remove(raw_rec)

    print(f"[MIC] Recording live ambient sound from tablet mic for {duration_sec} seconds...")
    subprocess.run(
        ["termux-microphone-record", "-f", raw_rec, "-l", str(int(duration_sec))],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    time.sleep(duration_sec + 0.8)
    subprocess.run(["termux-microphone-record", "-q"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    if os.path.exists(raw_rec) and os.path.getsize(raw_rec) > 256:
        subprocess.run(
            ["ffmpeg", "-y", "-i", raw_rec, "-ac", "1", "-ar", str(fs), "-sample_fmt", "s16", wav_rec],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            check=True,
        )
        return wav_rec
    return None


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(base_dir, "output")
    os.makedirs(out_dir, exist_ok=True)

    yt_clip = os.path.join(os.path.dirname(base_dir), "simulator", "input_youtube.wav")
    input_path = None

    if "--record-mic" in sys.argv:
        input_path = capture_live_microphone_wav(duration_sec=3, fs=FS)
    elif len(sys.argv) > 1 and not sys.argv[1].startswith("--") and os.path.exists(sys.argv[1]):
        input_path = sys.argv[1]
    elif os.path.exists(yt_clip):
        input_path = yt_clip

    if input_path:
        with wave.open(input_path, "rb") as wf:
            fs_in = wf.getframerate()
            n_ch = wf.getnchannels()
            raw = wf.readframes(min(wf.getnframes(), 6 * fs_in))
        pcm = np.frombuffer(raw, dtype=np.int16).astype(np.float64)
        if n_ch > 1:
            pcm = pcm.reshape(-1, n_ch).mean(axis=1)
        if fs_in != FS:
            pcm = signal.resample_poly(pcm, FS, fs_in)
        waveform = 0.85 * pcm / (np.max(np.abs(pcm)) + 1e-12)
        source_name = os.path.basename(input_path)
    else:
        t = np.arange(4 * FS, dtype=np.float64) / FS
        waveform = (
            0.45 * np.cos(2 * np.pi * 65.0 * t + 0.4)
            + 0.25 * np.cos(2 * np.pi * 130.0 * t - 0.8)
            + 0.15 * np.cos(2 * np.pi * 195.0 * t + 1.2)
        )
        source_name = "Precision Multi-Tone Test (65 Hz, 130 Hz, 195 Hz)"

    details, budget, inc_wave, opp_wave, res_wave = compute_zero_error_cancellation(waveform)

    print("=" * 78)
    print(" ACOUSTIC PHASE & TIMING CALCULATOR (feed-forward anti-noise)")
    print("=" * 78)
    print(f" Audio Source Analyzed : {source_name}")
    print(f" Sample Rate / Duration: {details['sample_rate_hz']} Hz | {details['duration_sec']} sec")
    print(f" Incoming Level        : {details['level_dbfs']} dBFS (RMS: {details['rms_level']})")
    print("-" * 78)
    print(" [1] DETECTED FREQUENCY & PHASE COMPONENTS:")
    print(f" {'#':<3} | {'Frequency':<12} | {'Period (T)':<12} | {'Wavelength':<12} | {'In Phase':<11} | {'Opposite (180°)'}")
    print("-" * 78)
    for c in details["components"]:
        print(
            f" {c['index']:<3} | {c['freq_hz']:>8.3f} Hz | {c['period_ms']:>8.3f} ms | "
            f"{c['wavelength_cm']:>8.2f} cm | {c['phase_deg']:>+8.2f}°  | {c['opposite_phase_deg']:>+8.2f}°"
        )

    print("-" * 78)
    print(" [2] TIMING BUDGET (MIC -> DSP -> SPEAKER):")
    print(f"   • Mic-to-Speaker Distance       : {budget['mic_to_speaker_distance_cm']:.2f} cm (Speed of Sound = {budget['speed_of_sound_m_s']} m/s)")
    print(f"   • 1. Acoustic Air Travel Time   : {budget['1_acoustic_travel_time_us']:>10.3f} us")
    print(f"   • 2. numpy negation time        : {budget['2_python_dsp_per_sample_us']:>10.4f} us / sample (not a real-time DSP figure)")
    print(f"   • 3. Hardware DAC + Amp Delay   : {budget['3_hardware_dac_amp_delay_us']:>10.3f} us")
    print(f"   • 4. Digital hold time          : {budget['4_exact_digital_wait_time_us']:>10.3f} us ({budget['exact_wait_samples_at_48khz']:.4f} samples @ 48 kHz)")
    print(f"   • 5. Polarity multiplier        : {budget['polarity_multiplier']:.1f}x")
    print("-" * 78)
    print(" [3] RESIDUAL AT THE SPEAKER PLANE (one point, not at a listener):")
    print(f"   • Incoming wave RMS             : {budget['incoming_rms_at_speaker']:.6f}")
    print(f"   • Perfect-path residual         : {budget['ideal_identity_rms']:.1e}  <- zero by construction (x - x), not a result")
    print(f"   • Assumed speaker-path errors   : gain {100 * budget['assumed_gain_error']:.0f}%, timing {budget['assumed_delay_error_us']:.0f} us")
    print(f"   • Residual with those errors    : {budget['residual_error_rms']:.6f}  ({budget['attenuation_db']:.1f} dB)")
    print("   • Causality: this uses the whole recording; a live system can only use past")
    print("     samples. See simulator/mismatch_sweep.py for the achievable range.")
    print("=" * 78)

    # Save output WAV files
    p_inc = os.path.join(out_dir, "1_captured_incoming_wave.wav")
    p_opp = os.path.join(out_dir, "2_exact_opposite_phase_speaker_wave.wav")
    p_res = os.path.join(out_dir, "3_residual_with_assumed_errors.wav")
    p_stereo = os.path.join(out_dir, "4_stereo_L_incoming_R_opposite.wav")

    save_wav_mono(p_inc, inc_wave)
    save_wav_mono(p_opp, opp_wave)
    save_wav_mono(p_res, res_wave)
    save_wav_stereo(p_stereo, inc_wave, opp_wave)

    print("\nSaved Precision Audio Files:")
    print(f"  1. Incoming Wave          : {p_inc}")
    print(f"  2. Opposite-Phase Output  : {p_opp}  <-- Sent directly to Speaker")
    print(f"  3. Residual w/ errors     : {p_res}")
    print(f"  4. Stereo Dual-Channel    : {p_stereo} (Left = Incoming, Right = -Incoming)")

    # Attempt to play the opposite-phase wave via termux-media-player if requested
    if "--play" in sys.argv:
        subprocess.run(["termux-media-player", "play", p_opp], check=False)


if __name__ == "__main__":
    main()
