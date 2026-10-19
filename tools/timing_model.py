"""
Delay budget for the open-window feed-forward ANC system, with strict bookkeeping.

Rule of this module: every term is one of
  calculated  - derived from a formula below with stated inputs,
  measured    - supplied from a measurement (e.g. the firmware's ISR timer),
  supplied    - typed in by the user (datasheet value, scope reading, ...),
  NOT SUPPLIED- unknown. It is never replaced by a guess. While anything is NOT SUPPLIED the
                result is a BOUND (margin <= x, break-even distance >= y), not a single number.

Definitions
-----------
    head start  H = (ref->error path - speaker->error path) / c
                  = d / c      when mic, speaker and error mic are collinear with the noise's
                               direction of travel (d = ref mic to speaker distance)
    delay       T = sum of the terms below
    margin      M = H - T       (M >= 0: the controller can react in time;
                                 M < 0: it must predict the noise |M| ahead)
    break-even  d_min = c * T   (the mic-to-speaker distance at which M = 0)

Terms and where they come from
------------------------------
  input / output filter : low-frequency (DC) group delay of the filter. For the low-pass filters
                          used here the group delay is largest at DC, so this is an upper bound
                          over the band. Unloaded filter formulas (no source/load impedance).
        1-pole RC               RC
        n-section RC ladder     RC * n(n+1)/2          (first moment of the impulse response)
        2nd-order Butterworth   sqrt(2) / (2*pi*fc)
  zero-order hold       : 1 / (2*fs). The ZOH has exactly linear phase, so T/2 holds at every
                          frequency below fs (assumes the DAC is written once per tick).
  ISR                   : time from the start of the sample tick to the end of the control step.
                          It already contains the ADC conversions, the filter maths and the SPI
                          write. Using the end of the step is an upper bound on sample->DAC time
                          because the DAC is written before the weight update.
  DAC settling          : DAC output settling after the SPI write (datasheet value).
  amplifier             : power-amp group delay (scope measurement).
  wiring                : length / v_cable. v_cable is an ASSUMPTION (about 2e8 m/s for ordinary
                          cable; slower in some cable, faster in none).

Speed of sound: c = sqrt(gamma * R * T / M) for dry air as an ideal gas (gamma 1.4,
M 0.0289647 kg/mol). Humidity is ignored. 20 C gives 343.23 m/s.

Not modelled (and why that matters)
-----------------------------------
  * Arrival direction: for a plane wave arriving at angle theta to the mic-speaker line,
    H = d*cos(theta)/c is shorter. Collinear is the best case.
  * Mic and speaker are treated as points. Use the acoustic centres.
  * The sealed woofer's low-frequency roll-off is minimum phase: it limits the gain available
    below its corner (cone excursion), not causality, so it is not a delay term here.
"""
import math

GAMMA_AIR = 1.4
R_GAS = 8.314462618        # J/(mol K)
M_AIR = 0.0289647          # kg/mol, dry air
V_CABLE_ASSUMED = 2.0e8    # m/s, ASSUMED signal speed in cable


def speed_of_sound(temp_c=20.0):
    """Speed of sound in dry air, ideal gas, m/s."""
    return math.sqrt(GAMMA_AIR * R_GAS * (temp_c + 273.15) / M_AIR)


# ----------------------------------------------------------------------------
# Filters
# ----------------------------------------------------------------------------
class Filter:
    """Low-pass filter: DC group delay, magnitude response and a human-readable description."""

    def __init__(self, description, dc_delay_s, gain_fn, fc_hz=None):
        self.description = description
        self.dc_delay_s = dc_delay_s
        self._gain = gain_fn
        self.fc_hz = fc_hz            # -3 dB frequency, if known in closed form

    def gain_db(self, f_hz):
        return 20.0 * math.log10(self._gain(f_hz))


def rc1(r_ohm, c_farad):
    tau = r_ohm * c_farad
    return Filter(f"1-pole RC, R={r_ohm:g} ohm, C={c_farad * 1e9:g} nF (unloaded)", tau,
                  lambda f: 1.0 / math.sqrt(1.0 + (2 * math.pi * f * tau) ** 2),
                  fc_hz=1.0 / (2 * math.pi * tau))


def rc_ladder2(r_ohm, c_farad):
    tau = r_ohm * c_farad

    def gain(f):
        x = 2 * math.pi * f * tau
        return 1.0 / math.hypot(1.0 - x * x, 3.0 * x)

    # -3 dB point of 1/(1 - x^2 + j3x): |H|^2 = 1/((1-x^2)^2 + 9x^2) = 1/2  ->  x^4 + 7x^2 - 1 = 0
    x3 = math.sqrt((-7.0 + math.sqrt(49.0 + 4.0)) / 2.0)
    return Filter(f"2-section RC ladder, R={r_ohm:g} ohm, C={c_farad * 1e9:g} nF each (unloaded)",
                  3.0 * tau, gain, fc_hz=x3 / (2 * math.pi * tau))


def butterworth2(fc_hz):
    return Filter(f"2nd-order Butterworth, fc={fc_hz:g} Hz",
                  math.sqrt(2.0) / (2 * math.pi * fc_hz),
                  lambda f: 1.0 / math.sqrt(1.0 + (f / fc_hz) ** 4), fc_hz=fc_hz)


def sallen_key_unity(r1_ohm, r2_ohm, c1_farad, c2_farad):
    """
    Unity-gain Sallen-Key low-pass: R1, R2 in series, C1 from the R1/R2 node to the op-amp output,
    C2 from the (+) input to analog ground. H = 1 / (s^2 R1 R2 C1 C2 + s C2 (R1 + R2) + 1).
    DC group delay = C2 (R1 + R2) exactly (the s^1 coefficient). Ideal op-amp.
    """
    w0 = 1.0 / math.sqrt(r1_ohm * r2_ohm * c1_farad * c2_farad)
    b1 = c2_farad * (r1_ohm + r2_ohm)
    b2 = r1_ohm * r2_ohm * c1_farad * c2_farad

    def gain(f):
        w = 2 * math.pi * f
        return 1.0 / math.hypot(1.0 - b2 * w * w, b1 * w)

    # -3 dB frequency from |H|^2 = 1/2: b2^2 x^2 + (b1^2 - 2 b2) x - 1 = 0 with x = w^2
    a_, b_, c_ = b2 * b2, b1 * b1 - 2.0 * b2, -1.0
    x = (-b_ + math.sqrt(b_ * b_ - 4 * a_ * c_)) / (2 * a_)
    return Filter(f"Sallen-Key unity gain, R1=R2={r1_ohm:g} ohm, C1={c1_farad * 1e9:g} nF, C2={c2_farad * 1e9:g} nF",
                  b1, gain, fc_hz=math.sqrt(x) / (2 * math.pi))


# preset name -> (input filter, output filter)
PRESETS = {
    "original-rc": (rc_ladder2(1500.0, 220e-9), rc1(1500.0, 220e-9)),
    "butterworth-1k": (butterworth2(1000.0), butterworth2(1000.0)),
    "sk-11k-20n-10n": (sallen_key_unity(11e3, 11e3, 20e-9, 10e-9), sallen_key_unity(11e3, 11e3, 20e-9, 10e-9)),
    "rc1-3k3-10n": (rc1(3300.0, 10e-9), rc1(3300.0, 10e-9)),
    "none": (None, None),
}


def zoh_delay_s(fs_hz):
    return 0.5 / fs_hz


# ----------------------------------------------------------------------------
# Budget
# ----------------------------------------------------------------------------
def head_start_s(ref_to_spk_m, spk_to_err_m, ref_to_err_m=None, temp_c=20.0):
    if ref_to_err_m is None:
        ref_to_err_m = ref_to_spk_m + spk_to_err_m       # collinear
    return (ref_to_err_m - spk_to_err_m) / speed_of_sound(temp_c)


def compute_budget(ref_to_spk_m=0.35, spk_to_err_m=0.25, ref_to_err_m=None,
                   in_filter=None, out_filter=None,
                   in_filter_s=None, out_filter_s=None,
                   fs_hz=4000.0, isr_s=None, isr_status="supplied",
                   dac_settle_s=None, amp_s=None, wire_m=0.0, extra_s=0.0, temp_c=20.0):
    """
    Return a dict. None for isr_s / dac_settle_s / amp_s means NOT SUPPLIED (unknown).
    Filter delays come from `in_filter`/`out_filter` objects, or from explicit
    `in_filter_s`/`out_filter_s` (seconds) when you measured them.
    """
    c = speed_of_sound(temp_c)
    head = head_start_s(ref_to_spk_m, spk_to_err_m, ref_to_err_m, temp_c)

    def filt_term(name, flt, explicit):
        if explicit is not None:
            return (name, explicit, "supplied")
        if flt is not None:
            return (name, flt.dc_delay_s, "calculated")
        return (name, 0.0, "supplied")       # 'none' preset: the user states there is no filter

    terms = [
        filt_term("input filter", in_filter, in_filter_s),
        filt_term("output filter", out_filter, out_filter_s),
        ("zero-order hold (1/(2 fs))", zoh_delay_s(fs_hz), "calculated"),
        ("ISR (ADC + maths + SPI write)", isr_s, isr_status if isr_s is not None else "NOT SUPPLIED"),
        ("DAC output settling", dac_settle_s, "supplied" if dac_settle_s is not None else "NOT SUPPLIED"),
        ("power amplifier", amp_s, "supplied" if amp_s is not None else "NOT SUPPLIED"),
        (f"wiring ({wire_m:g} m at an assumed {V_CABLE_ASSUMED:.1e} m/s)", wire_m / V_CABLE_ASSUMED, "calculated"),
    ]
    if extra_s:
        terms.append(("other delay", extra_s, "supplied"))

    known = sum(v for _, v, _ in terms if v is not None)
    missing = [n for n, v, _ in terms if v is None]
    complete = not missing
    upper_margin = head - known
    out = {
        "c_m_s": c, "temp_c": temp_c, "fs_hz": fs_hz,
        "head_start_s": head, "terms": terms,
        "known_total_s": known, "missing": missing, "complete": complete,
        "margin_s": upper_margin if complete else None,
        "margin_upper_bound_s": upper_margin,
        "d_min_m": c * known if complete else None,
        "d_min_lower_bound_m": c * known,
    }
    if upper_margin < 0:
        out["causal"] = False            # unknown terms can only add delay, so this is definite
    elif complete:
        out["causal"] = True
    else:
        out["causal"] = None             # undetermined until the missing terms are supplied
    return out


def anti_alias_report(flt, fs_hz, band_max_hz=300.0):
    """Exact magnitude numbers for judging aliasing; no verdict is made here."""
    lines = []
    if flt is None:
        return lines
    nyq = fs_hz / 2.0
    if flt.fc_hz is not None and flt.fc_hz >= nyq:
        lines.append(f"  WARNING: -3 dB frequency {flt.fc_hz:.0f} Hz is at or above the Nyquist frequency "
                     f"{nyq:.0f} Hz: this filter does not protect against aliasing at fs = {fs_hz:.0f} Hz")
    rows = [
        (f"gain at {band_max_hz:.0f} Hz (in-band droop)", band_max_hz),
        (f"gain at Nyquist {nyq:.0f} Hz", nyq),
        (f"gain at fs - {band_max_hz:.0f} Hz = {fs_hz - band_max_hz:.0f} Hz (aliases onto {band_max_hz:.0f} Hz)", fs_hz - band_max_hz),
    ]
    for label, f in rows:
        lines.append(f"  {label:<62}{flt.gain_db(f):9.3f} dB")
    return lines


def prediction_phase_deg(shortfall_s, freq_hz):
    return 360.0 * freq_hz * max(0.0, shortfall_s)


def band_residual_db(f_lo_hz, f_hi_hz, delay_err_s, gain=1.0):
    """
    Exact residual (dB re uncancelled) for noise with a flat spectrum over [f_lo, f_hi] when the
    anti-noise has `gain` and is late by `delay_err_s`: 10*log10( 1 + g^2 - 2 g * mean(cos(w tau)) ).
    """
    if delay_err_s == 0.0:
        mean_cos = 1.0
    else:
        w1, w2 = 2 * math.pi * f_lo_hz, 2 * math.pi * f_hi_hz
        mean_cos = (math.sin(w2 * delay_err_s) - math.sin(w1 * delay_err_s)) / (delay_err_s * (w2 - w1))
    power = 1.0 + gain * gain - 2.0 * gain * mean_cos
    return 10.0 * math.log10(max(power, 1e-30))


def format_budget(b, title="DELAY BUDGET"):
    lines = [title, "-" * len(title)]
    lines.append(f"  speed of sound at {b['temp_c']:g} C (dry air)      : {b['c_m_s']:10.3f} m/s")
    lines.append(f"  acoustic head start H                 : {b['head_start_s'] * 1e6:10.1f} us")
    lines.append("  delay terms:")
    for name, v, status in b["terms"]:
        val = "      --      " if v is None else f"{v * 1e6:10.1f} us"
        lines.append(f"    {name:<46} {val}   [{status}]")
    lines.append(f"  total of the terms that are known     : {b['known_total_s'] * 1e6:10.1f} us")
    if b["complete"]:
        lines.append(f"  margin M = H - T                      : {b['margin_s'] * 1e6:+10.1f} us")
        lines.append(f"  break-even distance d_min = c T       : {b['d_min_m'] * 100:10.2f} cm")
    else:
        lines.append(f"  NOT SUPPLIED: {', '.join(b['missing'])}")
        lines.append(f"  margin M                              <= {b['margin_upper_bound_s'] * 1e6:+9.1f} us  (upper bound)")
        lines.append(f"  break-even distance d_min             >= {b['d_min_lower_bound_m'] * 100:9.2f} cm  (lower bound)")
    if b["causal"] is True:
        lines.append("  verdict: the controller can react in time (all terms supplied)")
    elif b["causal"] is False:
        short = -b["margin_upper_bound_s"]
        lines.append(f"  verdict: the controller must predict the noise at least {short * 1e6:.1f} us ahead")
    else:
        lines.append("  verdict: undetermined until the missing terms are supplied")
    return "\n".join(lines)
