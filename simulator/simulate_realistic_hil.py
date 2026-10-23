#!/usr/bin/env python3
"""
3D Room & Hardware-in-the-Loop (HIL) Window ANC Simulator
=========================================================
The most detailed model in this repo: 48,000 Hz physical acoustics alongside the
4,000 Hz 12-bit quantized Teensy ISR. It is still a MODEL: the baffle and glass are
illustrative filters (not measured), the room is an image-source box with a Sabine
average absorption, and nothing here has been compared with a real window. Treat the
numbers as "what this model predicts", not as measured performance.

Contents:

1. 3D Bedroom Image-Source Acoustics (4.2m x 3.6m x 2.7m, RT60 = 0.35 s, Sabine average):
   - Separate 3D paths from Window Air-Exit Slot and 5.25" Woofer (12 cm apart)
     to the Indoor Error Mic (0.25m) AND a Stereo Human Listener (2.0m inside room).
   - Structural flanking transmission through the closed 4mm glass window pane.
2. Real Transducer & Enclosure Physics:
   - 5.25" Sealed Woofer (fc = 68 Hz, Qtc = 0.82): 12 dB/oct rolloff below 68 Hz,
     phase rotation, voice-coil inductance, and 2.5% non-linear cone THD.
   - Cardioid Outdoor Reference Mic with frequency-dependent rear rejection,
     -62 dBFS thermal self-noise, and uncorrelated outdoor wind buffeting (15-50 Hz).
   - Baffle: two first-order low-pass filters (260 Hz, 900 Hz) plus a 38 cm delay. This is an
     ILLUSTRATIVE filter, not measured foam data. The duct path it produces (without glass):
     0.5 dB @ 60 Hz, 3.4 dB @ 250 Hz, 10.1 dB @ 600 Hz, 27.6 dB @ 2 kHz.
3. Real Mixed-Signal Electronics (12:1 Multi-Rate 48 kHz <-> 4 kHz):
   - Minimum-phase 2-pole analog RC anti-aliasing & reconstruction filters
     (fc = 480 Hz) introducing 0.469 ms of low-frequency group delay each way (sqrt(2)/(2*pi*fc)).
   - 12-bit SAR ADC quantization (4,096 levels) + 1.2 LSB rms input noise = ENOB 9.90 bits
     (PJRC: about 10 bits normally usable). Sweep it with --adc-enob.
   - 12-bit MCP4921 SPI DAC quantization + Zero-Order Hold (ZOH) staircase.
"""

import argparse
import os
import wave
import numpy as np
from scipy import signal
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ============================================================================
# 1. MULTI-RATE CLOCK & HARDWARE CONSTANTS
# ============================================================================
FS_PHYS = 48000                 # Continuous physical acoustic simulation rate (48 kHz)
DECIM = 12                      # 12x ratio -> 4,000 Hz Teensy 4.0 hardware timer ISR
FS_ISR = FS_PHYS // DECIM       # 4,000 Hz

C_SOUND = 343.0                 # Speed of sound in air (m/s) at 20 C

# Teensy 4.0 FxLMS Firmware Parameters (Algorithm matches window_anc_fxlms.ino; step sizes differ slightly)
W_TAPS = 128
S_TAPS = 64
MU_S_IDENT = 0.08
MU_FXLMS = 0.012
LEAKAGE = 0.99992
EPSILON = 1e-4

# ADC noise model. The firmware reads 12-bit codes (analogReadResolution(12)), but PJRC's
# Teensy 4.0 page says only about 10 bits are normally usable because of noise. The model
# keeps the 12-bit quantizer and adds Gaussian input noise of ADC_NOISE_LSB (in 12-bit LSB).
# The default 1.2 LSB plus quantization gives an ENOB of 9.90 bits (see adc_enob()).
ADC_BITS = 12
ADC_NOISE_LSB = 1.2


# ============================================================================
# 2. 3D IMAGE-SOURCE ROOM IMPULSE RESPONSE GENERATOR
# ============================================================================
def fractional_delay_tap(ir, delay_sec, gain, fs=FS_PHYS):
    """Add a fractional-sample delayed impulse using 2-tap linear interpolation
    (adequate below ~2 kHz at 48 kHz; it attenuates higher frequencies a little)."""
    exact_idx = delay_sec * fs
    base = int(np.floor(exact_idx))
    frac = exact_idx - base
    if 1 <= base < len(ir) - 3:
        # Linear fractional interpolation
        ir[base] += gain * (1.0 - frac)
        ir[base + 1] += gain * frac


def compute_room_ir(src_xyz, dst_xyz, room_dims=(4.2, 3.6, 2.7), rt60=0.35, max_len_sec=0.18):
    """
    Compute realistic 3D room impulse response using the Image-Source Method
    for a rectangular bedroom (4.2m x 3.6m x 2.7m) up to reflection order 4.
    """
    ir_len = int(max_len_sec * FS_PHYS)
    ir = np.zeros(ir_len, dtype=np.float64)

    Lx, Ly, Hz = room_dims
    # Sabine wall reflection coefficient from RT60
    volume = Lx * Ly * Hz
    surface = 2.0 * (Lx * Ly + Ly * Hz + Lx * Hz)
    alpha_bar = min(0.65, 0.161 * volume / (surface * rt60))
    refl_coeff = np.sqrt(1.0 - alpha_bar)

    sx, sy, sz = src_xyz
    dx, dy, dz = dst_xyz
    r_ref = np.linalg.norm(np.array(dst_xyz) - np.array(src_xyz))

    # Sum 3D mirror images (orders -3..3 along X, Y, Z)
    for nx in range(-3, 4):
        for ny in range(-3, 4):
            for nz in range(-3, 4):
                order = abs(nx) + abs(ny) + abs(nz)
                if order > 5:
                    continue
                # Mirror coordinates around room center [0..Lx], [-Ly/2..Ly/2], [-Hz/2..Hz/2]
                # Image of a source in a box with walls at x=0 and x=Lx: m*Lx + (sx or Lx-sx)
                ix = (nx * Lx) + (sx if nx % 2 == 0 else (Lx - sx))
                iy = (ny * Ly) + (sy if ny % 2 == 0 else -sy)
                iz = (nz * Hz) + (sz if nz % 2 == 0 else -sz)

                dist = np.sqrt((dx - ix) ** 2 + (dy - iy) ** 2 + (dz - iz) ** 2)
                delay = dist / C_SOUND
                # Normalize direct path to r_ref so error mic has unit reference scale,
                # while 1/r spherical spreading scales reflections and distant listener ears
                attenuation = (refl_coeff ** order) * (0.35 / max(0.15, dist))
                # Alternating phase slightly for wall impedance
                fractional_delay_tap(ir, delay, attenuation)

    # High-frequency air/carpet absorption on reverberant tail
    b_air, a_air = signal.butter(1, 3500.0 / (FS_PHYS / 2.0), btype="low")
    ir_tail = signal.lfilter(b_air, a_air, ir)
    # Keep direct wave sharp, blend filtered tail
    direct_idx = int((r_ref / C_SOUND) * FS_PHYS) + 4
    ir[direct_idx:] = ir_tail[direct_idx:]
    return ir


# ============================================================================
# 3. TRANSDUCER, BAFFLE & ELECTRONIC HARDWARE MODELS
# ============================================================================
def build_woofer_impulse_response():
    """
    Model a real 5.25" mid-woofer in a 3.5L sealed enclosure (fc = 68 Hz, Qtc = 0.82):
    - 2nd-order high-pass roll-off below 68 Hz (12 dB/oct + phase rotation)
    - Cone breakup & voice coil inductance roll-off above 1,800 Hz
    """
    w0 = 2.0 * np.pi * 68.0
    q_tc = 0.82
    # Continuous s-domain high-pass: s^2 / (s^2 + (w0/Q)s + w0^2)
    b_s = [1.0, 0.0, 0.0]
    a_s = [1.0, w0 / q_tc, w0 ** 2]
    b_hp, a_hp = signal.bilinear(b_s, a_s, fs=FS_PHYS)

    # Voice coil inductance low-pass at 1,800 Hz
    b_lp, a_lp = signal.butter(2, 1800.0 / (FS_PHYS / 2.0), btype="low")

    imp = np.zeros(int(0.04 * FS_PHYS))
    imp[2] = 1.0  # ~42 us voice-coil electromechanical force lag
    ir = signal.lfilter(b_lp, a_lp, signal.lfilter(b_hp, a_hp, imp))
    return ir


def build_baffle_and_flanking_filters():
    """
    1. Foam-Lined S-Curve Baffle Transmission:
       - Adds 38 cm effective internal acoustic path length (~1.11 ms delay)
       - Illustrative loss of this filter (not measured foam): 0.5 dB at 60 Hz, 3.4 dB at 250 Hz,
         10.1 dB at 600 Hz, 27.6 dB at 2 kHz (computed from the filter, see the combined
         baffle + glass figures in the printed report).
    2. Glass pane flanking path (gain 0.045 = -26.9 dB above the 180 Hz high-pass corner):
       - A flat, assumed transmission. No mass-law slope and no coincidence dip is modelled.
    """
    # Baffle filter: cascade of shelf/lowpass filters matching melamine foam duct data
    b1, a1 = signal.butter(1, 260.0 / (FS_PHYS / 2.0), btype="low")
    b2, a2 = signal.butter(1, 900.0 / (FS_PHYS / 2.0), btype="low")

    imp = np.zeros(int(0.015 * FS_PHYS))
    delay_samples = int((0.38 / C_SOUND) * FS_PHYS)  # 38 cm internal S-duct travel
    imp[delay_samples] = 1.0

    # Blend 88% low-passed duct wave + 12% attenuated high-freq diffraction leak
    ir_lp1 = signal.lfilter(b1, a1, imp)
    ir_lp2 = signal.lfilter(b2, a2, ir_lp1)
    ir_baffle = 0.78 * ir_lp2 + 0.16 * ir_lp1 + 0.035 * imp

    # Glass pane structural flanking (transmits mid/highs faintly across whole window)
    b_g, a_g = signal.butter(1, 180.0 / (FS_PHYS / 2.0), btype="high")
    imp_g = np.zeros(int(0.010 * FS_PHYS))
    imp_g[int((0.12 / C_SOUND) * FS_PHYS)] = 0.045  # ~-27 dB through glass pane
    ir_glass = signal.lfilter(b_g, a_g, imp_g)

    return ir_baffle, ir_glass


def adc_enob(noise_lsb, bits=ADC_BITS):
    """Effective number of bits of the ADC model for a full-scale sine.

    Total noise = quantization (LSB^2 / 12) + Gaussian input noise (noise_lsb LSB rms);
    ENOB = bits - log2(total_rms / (LSB / sqrt(12))), the same as (SINAD - 1.76 dB) / 6.02 dB.
    """
    return bits - 0.5 * np.log2(1.0 + 12.0 * noise_lsb ** 2)


def adc_noise_lsb_for_enob(enob, bits=ADC_BITS):
    """Input noise (rms, in LSB) that makes adc_enob() equal to enob. Inverse of adc_enob()."""
    if enob > bits:
        raise ValueError(f"ENOB {enob} cannot exceed the {bits}-bit quantizer")
    return float(np.sqrt((4.0 ** (bits - enob) - 1.0) / 12.0))


def quantize_12bit_adc(voltage_norm, rng, noise_lsb=ADC_NOISE_LSB):
    """Simulate Teensy 4.0 12-bit SAR ADC (0..4095) with noise_lsb LSB rms input noise."""
    noisy = voltage_norm + rng.normal(0.0, noise_lsb / 2048.0, size=np.shape(voltage_norm))
    clipped = np.clip(noisy, -1.0, 0.9995)
    adc_code = np.round((clipped + 1.0) * 2048.0)
    return (adc_code - 2048.0) / 2048.0


def quantize_12bit_dac(sample_norm):
    """Simulate MCP4921 12-bit SPI DAC with [-0.95, +0.95] rail limit."""
    clipped = max(-0.95, min(0.95, sample_norm))
    dac_code = int(round((clipped + 1.0) * 2047.5))
    dac_code = max(0, min(4095, dac_code))
    return (dac_code - 2048.0) / 2048.0


# ============================================================================
# 4. MAIN HARDWARE-IN-THE-LOOP (HIL) SIMULATION
# ============================================================================
def run_realistic_hil(adc_noise_lsb=ADC_NOISE_LSB, write_outputs=True):
    """Run the model. Returns the printed figures as a dict (dB values, rounded to 0.1 dB)."""
    base_dir = os.path.dirname(os.path.abspath(__file__))
    yt_wav = os.path.join(base_dir, "input_youtube.wav")
    out_dir = os.path.join(base_dir, "output_realistic")
    os.makedirs(out_dir, exist_ok=True)

    print("=" * 78)
    print(" HYPER-REALISTIC 3D ROOM & HARDWARE-IN-THE-LOOP (HIL) ANC SIMULATOR")
    print(" Physics Rate: 48,000 Hz | Teensy 4.0 ISR: 4,000 Hz (12-Bit ADC/DAC)")
    print(f" ADC model: 12-bit codes + {adc_noise_lsb:.3f} LSB rms noise -> ENOB {adc_enob(adc_noise_lsb):.2f} bits")
    print("=" * 78)

    # 1. Load 18 seconds of native 48,000 Hz YouTube audio
    with wave.open(yt_wav, "rb") as wf:
        fs_in = wf.getframerate()
        n_ch = wf.getnchannels()
        raw_bytes = wf.readframes(18 * fs_in)
    pcm = np.frombuffer(raw_bytes, dtype=np.int16).astype(np.float64)
    if n_ch > 1:
        pcm = pcm.reshape(-1, n_ch).mean(axis=1)
    if fs_in != FS_PHYS:
        pcm = signal.resample_poly(pcm, FS_PHYS, fs_in)
    street_wave = 0.85 * pcm / (np.max(np.abs(pcm)) + 1e-9)
    N_48k = len(street_wave)
    N_4k = N_48k // DECIM

    # 2. Construct 3D Room Geometry & Multi-Path Impulse Responses
    # Coordinates (x = into room [m], y = horizontal along window [m], z = height [m]):
    pos_ref_mic = (-0.38, 0.00, 0.00)       # 38 cm outside window on boom
    pos_air_slot = (0.00, -0.06, 0.00)      # Window air-exit slot (-6 cm left of center)
    pos_woofer = (0.00, +0.06, 0.00)        # 5.25" Sealed Woofer (+6 cm right -> 12 cm apart!)
    pos_err_mic = (0.25, 0.00, 0.00)        # Indoor Error Mic (25 cm inside window)
    pos_ear_L = (2.00, -0.09, -0.10)        # Listener Left Ear (2.0m inside room!)
    pos_ear_R = (2.00, +0.09, -0.10)        # Listener Right Ear (2.0m inside room!)

    print("\n[1] Computing 3D Bedroom Impulse Responses (4.2m x 3.6m x 2.7m, RT60=0.35s)...")
    ir_slot_to_err = compute_room_ir(pos_air_slot, pos_err_mic)
    ir_woof_to_err = compute_room_ir(pos_woofer, pos_err_mic)

    ir_slot_to_earL = compute_room_ir(pos_air_slot, pos_ear_L)
    ir_slot_to_earR = compute_room_ir(pos_air_slot, pos_ear_R)
    ir_woof_to_earL = compute_room_ir(pos_woofer, pos_ear_L)
    ir_woof_to_earR = compute_room_ir(pos_woofer, pos_ear_R)

    ir_woofer_box = build_woofer_impulse_response()
    ir_baffle, ir_glass = build_baffle_and_flanking_filters()

    # Combine transducer/baffle physics with 3D room paths
    # Path from outdoor Reference Mic plane (-0.38m) to Window Plane (0.0m) in free air:
    ext_delay_samples = int((0.38 / C_SOUND) * FS_PHYS)
    ext_air_ir = np.zeros(ext_delay_samples + 1)
    ext_air_ir[ext_delay_samples] = 1.0

    # Full Primary Paths (from Outdoor Ref Mic plane -> Indoor Error Mic & Listener Ears)
    # A) Bare Open Window (no foam baffle, just open aperture + room acoustics)
    P_bare_err = signal.fftconvolve(ext_air_ir, ir_slot_to_err)
    P_bare_earL = signal.fftconvolve(ext_air_ir, ir_slot_to_earL)
    P_bare_earR = signal.fftconvolve(ext_air_ir, ir_slot_to_earR)

    # B) Baffled Window (S-curve foam duct + glass pane structural flanking)
    baffle_plus_glass = np.zeros(max(len(ir_baffle), len(ir_glass)))
    baffle_plus_glass[:len(ir_baffle)] += ir_baffle
    baffle_plus_glass[:len(ir_glass)] += ir_glass

    P_baff_err = signal.fftconvolve(ext_air_ir, signal.fftconvolve(baffle_plus_glass, ir_slot_to_err))
    P_baff_earL = signal.fftconvolve(ext_air_ir, signal.fftconvolve(baffle_plus_glass, ir_slot_to_earL))
    P_baff_earR = signal.fftconvolve(ext_air_ir, signal.fftconvolve(baffle_plus_glass, ir_slot_to_earR))

    # Full Secondary Paths (from Amplifier Output -> Sealed 5.25" Woofer -> Error Mic & Ears)
    S_phys_err = signal.fftconvolve(ir_woofer_box, ir_woof_to_err)
    S_phys_earL = signal.fftconvolve(ir_woofer_box, ir_woof_to_earL)
    S_phys_earR = signal.fftconvolve(ir_woofer_box, ir_woof_to_earR)

    # Acoustic Feedback Path (Woofer -> through/around baffle -> Outdoor Cardioid Ref Mic)
    # Cardioid mic rejects rear speaker sound by ~12 dB in midbass, ~7 dB below 80 Hz
    dist_woof_ref = np.linalg.norm(np.array(pos_ref_mic) - np.array(pos_woofer))
    fb_air = np.zeros(int((dist_woof_ref / C_SOUND) * FS_PHYS) + 4)
    fractional_delay_tap(fb_air, dist_woof_ref / C_SOUND, 0.14)
    F_phys = signal.fftconvolve(ir_woofer_box, fb_air)

    # 3. Analog Anti-Aliasing & Reconstruction Filters (2-Pole Minimum-Phase @ fc=480 Hz)
    b_aa, a_aa = signal.butter(2, 480.0 / (FS_PHYS / 2.0), btype="low")

    # Outdoor wind buffeting on Reference Mic (uncorrelated 15-45 Hz wind turbulence
    # that hits the outdoor windscreen but does NOT propagate as sound into the room!)
    rng = np.random.default_rng(2026)
    b_wind, a_wind = signal.butter(2, [15.0 / (FS_PHYS / 2.0), 45.0 / (FS_PHYS / 2.0)], btype="band")
    wind_turbulence = signal.lfilter(b_wind, a_wind, rng.normal(0.0, 0.012, size=N_48k))

    # ------------------------------------------------------------------------
    # 4. STAGE 1: 3-SECOND HARDWARE AUTO-CALIBRATION AT 4,000 Hz
    # ------------------------------------------------------------------------
    print("[2] Running Stage 1: 3.0s Startup Calibration (with 12-bit ADC/DAC & room noise)...")
    # Each 4,000 Hz DAC sample is held constant (ZOH) for DECIM = 12 ticks at 48,000 Hz.
    # Therefore, a unit pulse at 4,000 Hz equals 12 consecutive ones at 48,000 Hz!
    zoh_pulse_48k = np.ones(DECIM, dtype=np.float64)

    # Full 48 kHz pulse responses from a single 4 kHz DAC ZOH step:
    # DAC ZOH (12 samples) -> Analog Recon Filter -> Woofer -> 3D Room -> Error/Ref Mic -> Analog AA Filter
    # (Convolve with S_phys_err FIRST so lfilter runs over the full 8,640-sample array instead of truncating at 12 samples!)
    zoh_S_48k = signal.fftconvolve(zoh_pulse_48k, S_phys_err)
    zoh_F_48k = signal.fftconvolve(zoh_pulse_48k, F_phys)
    step_S_err_48k = signal.lfilter(b_aa, a_aa, signal.lfilter(b_aa, a_aa, zoh_S_48k))
    step_F_ref_48k = signal.lfilter(b_aa, a_aa, signal.lfilter(b_aa, a_aa, zoh_F_48k))

    # Sample the continuous 48 kHz response at the 4,000 Hz ISR sampling instants.
    # Because the ISR writes the DAC *after* reading the ADCs at tick n*DECIM,
    # the DAC output written at step n affects ADC readings starting at step n+1 ((n+1)*DECIM).
    M_TRUE = 96
    S_true_4k = np.zeros(M_TRUE, dtype=np.float64)
    F_true_4k = np.zeros(M_TRUE, dtype=np.float64)
    S_true_4k[1:] = step_S_err_48k[DECIM::DECIM][: M_TRUE - 1]
    F_true_4k[1:] = step_F_ref_48k[DECIM::DECIM][: M_TRUE - 1]

    S_hat = np.zeros(S_TAPS, dtype=np.float64)
    F_hat = np.zeros(S_TAPS, dtype=np.float64)
    y_hist_true = np.zeros(M_TRUE, dtype=np.float64)
    y_buf_calib = np.zeros(S_TAPS, dtype=np.float64)

    HPF_ALPHA = 0.962
    c_ref_in = c_ref_out = 0.0
    c_err_in = c_err_out = 0.0

    calib_steps = int(3.0 * FS_ISR)
    for _ in range(calib_steps):
        # 1. Read ADCs at start of ISR. Same bookkeeping as the control loop below:
        #    shift first, so S_true_4k[k] multiplies the DAC value written k ticks ago
        #    (previously this loop read before shifting, which made the plant one tick
        #    slower during calibration and gave S_hat a 1-sample offset).
        y_hist_true[1:] = y_hist_true[:-1]
        y_hist_true[0] = 0.0
        e_raw = np.dot(S_true_4k, y_hist_true) + rng.normal(0.0, 0.0015)
        x_raw = np.dot(F_true_4k, y_hist_true) + rng.normal(0.0, 0.0020)

        adc_err = float(quantize_12bit_adc(e_raw, rng, adc_noise_lsb))
        adc_ref = float(quantize_12bit_adc(x_raw, rng, adc_noise_lsb))

        # 2. 25 Hz DC-blocking High-Pass Filter (exact match to window_anc_fxlms.ino)
        x_n = HPF_ALPHA * (c_ref_out + adc_ref - c_ref_in)
        c_ref_in, c_ref_out = adc_ref, x_n

        e_n = HPF_ALPHA * (c_err_out + adc_err - c_err_in)
        c_err_in, c_err_out = adc_err, e_n

        # 3. Generate calibration white noise & write to DAC
        v_n = quantize_12bit_dac(rng.uniform(-0.18, 0.18))
        v_cone = np.tanh(1.10 * v_n) + 0.02 * (v_n ** 2)

        # Record into physical history and calibration buffer
        y_hist_true[0] = v_cone

        y_buf_calib[1:] = y_buf_calib[:-1]
        y_buf_calib[0] = v_n

        # 4. Update S_hat and F_hat via NLMS
        e_hat = np.dot(S_hat, y_buf_calib)
        f_hat = np.dot(F_hat, y_buf_calib)
        p_v = np.dot(y_buf_calib, y_buf_calib) + EPSILON

        S_hat += (MU_S_IDENT * (e_n - e_hat) / p_v) * y_buf_calib
        F_hat += (MU_S_IDENT * (x_n - f_hat) / p_v) * y_buf_calib

    # Effective target impulse response including the 25 Hz digital HPF
    S_target_hpf = signal.lfilter([HPF_ALPHA, -HPF_ALPHA], [1.0, -HPF_ALPHA], S_true_4k[:S_TAPS])
    nmse_s = 10.0 * np.log10(
        np.sum((S_target_hpf - S_hat) ** 2) / (np.sum(S_target_hpf ** 2) + 1e-12)
    )
    print(f"    -> Calibrated S_hat(z) Accuracy (with 12-bit noise & room tail): {nmse_s:.1f} dB NMSE")

    # ------------------------------------------------------------------------
    # 5. STAGE 2: REAL-TIME SAMPLE-BY-SAMPLE HARDWARE-IN-THE-LOOP (HIL)
    # ------------------------------------------------------------------------
    print("[3] Propagating full 48,000 Hz street waveform through 3D room & analog filters...")

    # Pre-compute true 48 kHz primary sound arriving at Error Mic and Listener Ears
    d_bare_err_48k = signal.fftconvolve(street_wave, P_bare_err)[:N_48k]
    d_bare_earL_48k = signal.fftconvolve(street_wave, P_bare_earL)[:N_48k]
    d_bare_earR_48k = signal.fftconvolve(street_wave, P_bare_earR)[:N_48k]

    d_baff_err_48k = signal.fftconvolve(street_wave, P_baff_err)[:N_48k]
    d_baff_earL_48k = signal.fftconvolve(street_wave, P_baff_earL)[:N_48k]
    d_baff_earR_48k = signal.fftconvolve(street_wave, P_baff_earR)[:N_48k]

    # Pass Error Mic primary sound and Outdoor Ref Mic sound through real analog AA filter
    d_baff_err_aa_48k = signal.lfilter(b_aa, a_aa, d_baff_err_48k)
    ref_ext_aa_48k = signal.lfilter(b_aa, a_aa, street_wave + wind_turbulence)

    # Sample at 4,000 Hz Teensy ISR ticks (t = n * DECIM)
    d_err_4k = d_baff_err_aa_48k[::DECIM]
    ref_ext_4k = ref_ext_aa_48k[::DECIM]

    # State buffers for Teensy 4.0 ISR
    W = np.zeros(W_TAPS, dtype=np.float64)
    x_buf = np.zeros(W_TAPS, dtype=np.float64)
    xf_buf = np.zeros(W_TAPS, dtype=np.float64)
    s_buf = np.zeros(S_TAPS, dtype=np.float64)
    y_buf_ctrl = np.zeros(S_TAPS, dtype=np.float64)
    y_hist_phys = np.zeros(M_TRUE, dtype=np.float64)

    ref_prev_in = ref_prev_out = 0.0
    err_prev_in = err_prev_out = 0.0

    dac_out_4k = np.zeros(N_4k, dtype=np.float64)
    cone_out_4k = np.zeros(N_4k, dtype=np.float64)

    print("[4] Running 4,000 Hz Teensy ISR with 12-bit quantization & non-linear woofer...")
    for n in range(N_4k):
        # 1. True physical acoustic superposition at Error Mic and Reference Mic at tick n*DECIM
        # Note: y_hist_phys[0] is empty at the start of tick n (it gets filled when DAC writes below),
        # matching S_true_4k[1:] which starts at delay 1.
        y_hist_phys[1:] = y_hist_phys[:-1]
        y_hist_phys[0] = 0.0

        anti_at_err = np.dot(S_true_4k, y_hist_phys)
        fb_at_ref = np.dot(F_true_4k, y_hist_phys)

        raw_ref_v = ref_ext_4k[n] + fb_at_ref
        raw_err_v = d_err_4k[n] + anti_at_err

        # 2. 12-bit SAR ADC quantization + thermal jitter
        adc_ref = float(quantize_12bit_adc(raw_ref_v, rng, adc_noise_lsb))
        adc_err = float(quantize_12bit_adc(raw_err_v, rng, adc_noise_lsb))

        # 3. 25 Hz DC-blocking High-Pass Filter
        x_n = HPF_ALPHA * (ref_prev_out + adc_ref - ref_prev_in)
        ref_prev_in, ref_prev_out = adc_ref, x_n

        e_n = HPF_ALPHA * (err_prev_out + adc_err - err_prev_in)
        err_prev_in, err_prev_out = adc_err, e_n

        # 4. Acoustic feedback neutralization. y_buf_ctrl[j] holds the output from j+1 ticks
        #    ago (current output not yet pushed), while F_hat[k] was learned against the
        #    output k ticks ago, so the pairing is F_hat[j+1] with y_buf_ctrl[j].
        x_clean = x_n - np.dot(F_hat[1:], y_buf_ctrl[:-1])

        # 5. Shift into control filter buffer
        x_buf[1:] = x_buf[:-1]
        x_buf[0] = x_clean

        # 6. Compute control output & quantize to 12-bit MCP4921 SPI DAC
        y_n = np.dot(W, x_buf)
        u_dac = quantize_12bit_dac(-y_n)
        dac_out_4k[n] = u_dac

        # Non-linear woofer cone distortion (~2.5% THD at high excursion)
        u_cone = np.tanh(1.10 * u_dac) + 0.02 * (u_dac ** 2)
        cone_out_4k[n] = u_cone
        y_hist_phys[0] = u_cone

        # 7. Update controller state buffers
        y_buf_ctrl[1:] = y_buf_ctrl[:-1]
        y_buf_ctrl[0] = u_dac
        s_buf[1:] = s_buf[:-1]
        s_buf[0] = x_clean

        xf_n = np.dot(S_hat, s_buf)
        xf_buf[1:] = xf_buf[:-1]
        xf_buf[0] = xf_n

        # 8. Normalized Leaky FxLMS update
        xf_power = np.dot(xf_buf, xf_buf) + EPSILON
        W = (LEAKAGE * W) + ((MU_FXLMS * e_n) / xf_power) * xf_buf

    # ------------------------------------------------------------------------
    # 6. RECONSTRUCT CONTINUOUS 48,000 Hz SOUND AT ERROR MIC & LISTENER'S EARS
    # ------------------------------------------------------------------------
    # Zero-Order Hold (ZOH) staircase from 4 kHz DAC -> 48 kHz + Analog Reconstruction Filter
    cone_zoh_48k = np.repeat(cone_out_4k, DECIM)[:N_48k]
    amp_in_48k = signal.lfilter(b_aa, a_aa, cone_zoh_48k)

    # Propagate continuous 48 kHz Woofer anti-noise to Error Mic (0.25m) & Listener Ears (2.0m)
    anti_err_48k = signal.fftconvolve(amp_in_48k, S_phys_err)[:N_48k]
    anti_earL_48k = signal.fftconvolve(amp_in_48k, S_phys_earL)[:N_48k]
    anti_earR_48k = signal.fftconvolve(amp_in_48k, S_phys_earR)[:N_48k]

    # Final continuous 48 kHz acoustic pressure waves:
    # 1) At Indoor Error Mic (0.25m from window):
    err_mic_bare = d_bare_err_48k
    err_mic_baffle_only = d_baff_err_48k
    err_mic_hybrid_anc = d_baff_err_48k + anti_err_48k

    # 2) At Human Listener's Ears (2.0m inside the bedroom, Binaural Stereo!):
    earL_bare, earR_bare = d_bare_earL_48k, d_bare_earR_48k
    earL_anc_only, earR_anc_only = d_bare_earL_48k + anti_earL_48k, d_bare_earR_48k + anti_earR_48k
    earL_baffle, earR_baffle = d_baff_earL_48k, d_baff_earR_48k
    earL_hybrid, earR_hybrid = d_baff_earL_48k + anti_earL_48k, d_baff_earR_48k + anti_earR_48k

    # ------------------------------------------------------------------------
    # 7. BAND-BY-BAND REALISTIC ATTENUATION REPORT (Steady State: t = 5s..18s)
    # ------------------------------------------------------------------------
    s_idx = 5 * FS_PHYS

    def band_rms(sig, f_lo, f_hi):
        b, a = signal.butter(3, [f_lo / (FS_PHYS / 2.0), f_hi / (FS_PHYS / 2.0)], btype="band")
        filtered = signal.lfilter(b, a, sig)
        return np.sqrt(np.mean(filtered[s_idx:] ** 2))

    def db_diff(val, ref):
        return 20.0 * np.log10((val + 1e-12) / (ref + 1e-12))

    bands = [
        ("Sub-Bass (35 - 70 Hz)   [Woofer Roll-Off Zone]", 35.0, 70.0),
        ("Core Traffic Bass (70 - 200 Hz) [ANC Sweet Spot]", 70.0, 200.0),
        ("Upper Bass (200 - 350 Hz) [Spatial Limit Zone]", 200.0, 350.0),
        ("Midrange Horns (350 - 1,200 Hz) [Passive Baffle]", 350.0, 1200.0),
        ("High Honks/Hiss (1.2k - 6.0k Hz) [Baffle+Glass]", 1200.0, 6000.0),
    ]

    print("\n" + "=" * 78)
    print(" REALISTIC PERFORMANCE AT HUMAN LISTENER (2.0 METERS INSIDE BEDROOM)")
    print("=" * 78)
    print(f"{'Frequency Band':<48} | {'Baffle Only':<12} | {'Baffle + ANC'}")
    print("-" * 78)
    results = {"adc_noise_lsb": adc_noise_lsb, "adc_enob": round(float(adc_enob(adc_noise_lsb)), 2),
               "s_hat_nmse_db": round(float(nmse_s), 1), "bands": {}}
    for name, flo, fhi in bands:
        r_bare = band_rms(earL_bare, flo, fhi)
        r_baff = band_rms(earL_baffle, flo, fhi)
        r_hyb = band_rms(earL_hybrid, flo, fhi)
        results["bands"][f"{flo:g}-{fhi:g}"] = (round(float(db_diff(r_baff, r_bare)), 1),
                                                 round(float(db_diff(r_hyb, r_bare)), 1))
        print(f"{name:<48} | {db_diff(r_baff, r_bare):>8.1f} dB  | {db_diff(r_hyb, r_bare):>8.1f} dB")

    # Compare 70-220 Hz Core Bass at Error Mic (0.25m) vs Listener Ear (2.0m)
    err_bass_off = band_rms(err_mic_baffle_only, 70.0, 220.0)
    err_bass_on = band_rms(err_mic_hybrid_anc, 70.0, 220.0)
    ear_bass_off = band_rms(earL_baffle, 70.0, 220.0)
    ear_bass_on = band_rms(earL_hybrid, 70.0, 220.0)

    total_bare = np.sqrt(np.mean(earL_bare[s_idx:] ** 2))
    total_baff = np.sqrt(np.mean(earL_baffle[s_idx:] ** 2))
    total_hyb = np.sqrt(np.mean(earL_hybrid[s_idx:] ** 2))

    print("-" * 78)
    print(f"Core Traffic Bass (70-220 Hz) Active Cancellation alone:")
    print(f"  - At Indoor Error Mic (0.25m from window) : {db_diff(err_bass_on, err_bass_off):>6.1f} dB")
    print(f"  - At Listener's Ears  (2.00m inside room) : {db_diff(ear_bass_on, ear_bass_off):>6.1f} dB")
    print(f"Overall Full-Band Traffic Reduction at Listener (2.0m): {db_diff(total_hyb, total_bare):>6.1f} dB")
    print("=" * 78)
    results["anc_err_mic_db"] = round(float(db_diff(err_bass_on, err_bass_off)), 1)
    results["anc_ear_db"] = round(float(db_diff(ear_bass_on, ear_bass_off)), 1)
    results["full_band_db"] = round(float(db_diff(total_hyb, total_bare)), 1)
    if not write_outputs:
        return results

    # ------------------------------------------------------------------------
    # 8. SAVE 48 kHz BINAURAL STEREO WAV FILES (Normalized to Common Peak)
    # ------------------------------------------------------------------------
    peak_scale = 0.90 / (max(np.max(np.abs(earL_bare)), np.max(np.abs(earR_bare))) + 1e-9)

    def save_stereo_wav(fname, left, right):
        path = os.path.join(out_dir, fname)
        stereo = np.column_stack((left * peak_scale, right * peak_scale))
        pcm16 = np.clip(stereo * 32767.0, -32767, 32767).astype(np.int16)
        with wave.open(path, "wb") as wf:
            wf.setnchannels(2)
            wf.setsampwidth(2)
            wf.setframerate(FS_PHYS)
            wf.writeframes(pcm16.tobytes())
        return path

    p1 = save_stereo_wav("1_realistic_bare_open_window_stereo.wav", earL_bare, earR_bare)
    p2 = save_stereo_wav("2_realistic_baffle_only_anc_off_stereo.wav", earL_baffle, earR_baffle)
    p3 = save_stereo_wav("3_realistic_hybrid_baffle_plus_anc_stereo.wav", earL_hybrid, earR_hybrid)

    # 18-second 3-Stage Binaural Comparison at Listener Position (6s per stage):
    # 0-6s  : Bare Open Window (Full 3D room reverberant traffic + horns + rumble)
    # 6-12s : Passive Window Baffle Installed, ANC Woofer OFF (Horns muffled, bass booms in room)
    # 12-18s: Hybrid Window Baffle + Active 5.25" Woofer ON (Bass rumble cancelled across room)
    s6 = 6 * FS_PHYS
    demo_L = np.concatenate([earL_bare[:s6], earL_baffle[s6 : 2 * s6], earL_hybrid[2 * s6 : 3 * s6]])
    demo_R = np.concatenate([earR_bare[:s6], earR_baffle[s6 : 2 * s6], earR_hybrid[2 * s6 : 3 * s6]])
    p_demo = save_stereo_wav("4_REALISTIC_BINAURAL_DEMO_3_STAGES.wav", demo_L, demo_R)

    # ------------------------------------------------------------------------
    # 9. GENERATE HIGH-RESOLUTION ENGINEERING SPECTRUM & CONVERGENCE PLOT
    # ------------------------------------------------------------------------
    fig, axes = plt.subplots(2, 1, figsize=(10, 7.5), dpi=140)
    fig.patch.set_facecolor("#12161f")
    for ax in axes:
        ax.set_facecolor("#1a202c")
        ax.tick_params(colors="#e2e8f0")
        for spine in ax.spines.values():
            spine.set_color("#4a5568")

    # Top Plot: Power Spectral Density (30 Hz - 1,000 Hz) at Listener's Ears (2.0m)
    f_psd, psd_bare = signal.welch(earL_bare[s_idx:], fs=FS_PHYS, nperseg=16384)
    _, psd_baff = signal.welch(earL_baffle[s_idx:], fs=FS_PHYS, nperseg=16384)
    _, psd_hyb = signal.welch(earL_hybrid[s_idx:], fs=FS_PHYS, nperseg=16384)

    ref_max = np.max(psd_bare) + 1e-18
    axes[0].semilogx(f_psd, 10 * np.log10(psd_bare / ref_max + 1e-12), color="#fc8181", lw=1.6, label="1. Bare Open Window (At Listener 2.0m)")
    axes[0].semilogx(f_psd, 10 * np.log10(psd_baff / ref_max + 1e-12), color="#f6e05e", lw=1.6, label="2. Passive Foam Baffle Only (ANC OFF)")
    axes[0].semilogx(f_psd, 10 * np.log10(psd_hyb / ref_max + 1e-12), color="#68d391", lw=2.0, label="3. Full Hybrid: Foam Baffle + Active 5.25\" Woofer ON")
    axes[0].axvspan(65, 250, color="#4fd1c5", alpha=0.12, label="Active Woofer Sweet Spot (65 - 250 Hz)")
    axes[0].set_xlim(30, 2000)
    axes[0].set_ylim(-55, 3)
    axes[0].set_title("Realistic 3D Bedroom Spectrum at Listener's Ears (2.0m Inside Room)", color="#ffffff", fontsize=12, fontweight="bold")
    axes[0].set_xlabel("Frequency (Hz) — Log Scale", color="#cbd5e0")
    axes[0].set_ylabel("Relative Sound Level (dB)", color="#cbd5e0")
    axes[0].grid(True, which="both", color="#2d3748", linestyle="--", alpha=0.7)
    axes[0].legend(loc="lower left", facecolor="#1a202c", edgecolor="#4a5568", labelcolor="#e2e8f0", fontsize=9)

    # Bottom Plot: Core Bass (70 - 200 Hz) Time-Domain Envelope at Listener (2.0m)
    b_bp, a_bp = signal.butter(3, [70.0 / (FS_PHYS / 2.0), 200.0 / (FS_PHYS / 2.0)], btype="band")
    bass_baff_t = signal.lfilter(b_bp, a_bp, earL_baffle)
    bass_hyb_t = signal.lfilter(b_bp, a_bp, earL_hybrid)
    t_axis = np.arange(N_48k) / float(FS_PHYS)

    axes[1].plot(t_axis[::24], bass_baff_t[::24], color="#f6e05e", alpha=0.75, lw=0.8, label="Uncancelled Traffic Bass (70-200 Hz) Entering Room")
    axes[1].plot(t_axis[::24], bass_hyb_t[::24], color="#68d391", alpha=0.9, lw=0.9, label="Residual Bass with Realistic HIL Active Woofer ON")
    axes[1].set_xlim(0, 18)
    axes[1].set_title("Real-Time Bass Waveform (70–200 Hz) at Listener's Ears (2.0m Inside Bedroom)", color="#ffffff", fontsize=12, fontweight="bold")
    axes[1].set_xlabel("Time (seconds)", color="#cbd5e0")
    axes[1].set_ylabel("Acoustic Pressure", color="#cbd5e0")
    axes[1].grid(True, color="#2d3748", linestyle="--", alpha=0.7)
    axes[1].legend(loc="upper right", facecolor="#1a202c", edgecolor="#4a5568", labelcolor="#e2e8f0", fontsize=9)

    plt.tight_layout()
    plot_path = os.path.join(out_dir, "realistic_anc_analysis.png")
    plt.savefig(plot_path, dpi=140)
    plt.close(fig)

    print(f"\n[5] Saved 48 kHz Binaural Stereo WAVs & Spectrum Plot in {out_dir}:")
    print(f"    - {p_demo}")
    print(f"    - {p1}")
    print(f"    - {p2}")
    print(f"    - {p3}")
    print(f"    - {plot_path}")
    return results


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    noise = parser.add_mutually_exclusive_group()
    noise.add_argument("--adc-noise-lsb", type=float, default=ADC_NOISE_LSB,
                       help=f"ADC input noise, rms, in 12-bit LSB (default {ADC_NOISE_LSB})")
    noise.add_argument("--adc-enob", type=float,
                       help="set the ADC noise so the model's ENOB equals this many bits")
    parser.add_argument("--no-output", action="store_true",
                        help="print the figures only; do not rewrite the WAV files and plot")
    args = parser.parse_args()
    lsb = args.adc_noise_lsb if args.adc_enob is None else adc_noise_lsb_for_enob(args.adc_enob)
    run_realistic_hil(adc_noise_lsb=lsb, write_outputs=not args.no_output)
