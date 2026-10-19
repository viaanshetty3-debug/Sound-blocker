"""
Delay / causality model for the open-window feed-forward ANC system.

One place for the arithmetic, shared by tools/timing_tool.py and simulator/mismatch_sweep.py.

The question it answers: how much time is there between "the reference mic hears the noise"
and "the anti-noise must already be at the error mic", and how much of it do the electronics use?

    head_start = (ref->error acoustic path) - (speaker->error acoustic path)
    margin     = head_start - (sum of electronics / filter delays)

margin >= 0: the controller can react in time. margin < 0: it must PREDICT the noise
|margin| seconds ahead. Prediction works for tonal / periodic content and degrades quickly for
broadband random noise, so a negative margin limits broadband reduction badly (it is a soft limit,
not a switch from "works" to "does nothing"; see simulator/mismatch_sweep.py for numbers).

Assumptions (state them when quoting a result):
  * Reference mic, speaker and error mic lie on a line along the arrival direction of the noise
    (best case; any other arrival angle shortens the head start). Pass ref_to_err_m to override.
  * Filter delays are the low-frequency group-delay limits (close to the in-band value for the
    70-300 Hz band of interest).
  * Output zero-order hold costs half a sample period on average.
  * The sealed woofer's low-frequency roll-off is NOT counted as delay: it is minimum phase, so
    it limits the gain you can get below the woofer corner (excursion), not causality.
"""
import math

C_SOUND = 343.0   # m/s at about 20 C


def ladder_rc_delay_s(n_sections, r_ohm, c_farad):
    """Elmore delay of n identical, unloaded RC sections in a ladder: RC * n(n+1)/2."""
    return r_ohm * c_farad * n_sections * (n_sections + 1) / 2.0


def butterworth2_delay_s(fc_hz):
    """Low-frequency group delay of a 2nd-order Butterworth low-pass: sqrt(2) / (2*pi*fc)."""
    return math.sqrt(2.0) / (2.0 * math.pi * fc_hz)


def zoh_delay_s(fs_hz):
    """Average delay of a zero-order-hold DAC output: half a sample period."""
    return 0.5 / fs_hz


PRESETS = {
    # name: (input filter delay s, output filter delay s, description)
    "original-rc": (
        ladder_rc_delay_s(2, 1500.0, 220e-9),
        ladder_rc_delay_s(1, 1500.0, 220e-9),
        "original guide: 2-section passive RC in (1.5k/220nF), 1-section RC out",
    ),
    "butterworth-1k": (
        butterworth2_delay_s(1000.0),
        butterworth2_delay_s(1000.0),
        "Sallen-Key 2nd-order Butterworth, fc = 1 kHz, in and out",
    ),
    "none": (0.0, 0.0, "no analog filter delay (what the toy simulators assume)"),
}


def head_start_s(ref_to_spk_m, spk_to_err_m, ref_to_err_m=None):
    """Acoustic head start in seconds."""
    if ref_to_err_m is None:
        ref_to_err_m = ref_to_spk_m + spk_to_err_m      # collinear best case
    return (ref_to_err_m - spk_to_err_m) / C_SOUND


def compute_budget(ref_to_spk_m=0.35, spk_to_err_m=0.25, ref_to_err_m=None,
                   in_filter_s=0.0, out_filter_s=0.0, fs_hz=4000.0,
                   isr_exec_s=0.0, extra_s=0.0):
    """Return a dict describing the delay budget. All times in seconds."""
    head = head_start_s(ref_to_spk_m, spk_to_err_m, ref_to_err_m)
    items = [
        ("input filter", in_filter_s),
        ("output filter", out_filter_s),
        ("zero-order hold (T/2)", zoh_delay_s(fs_hz)),
        ("ISR execution", isr_exec_s),
        ("other / extra", extra_s),
    ]
    total = sum(v for _, v in items)
    margin = head - total
    return {
        "head_start_s": head,
        "items": items,
        "total_s": total,
        "margin_s": margin,
        "causal": margin >= 0.0,
        "fs_hz": fs_hz,
    }


def prediction_phase_deg(margin_s, freq_hz):
    """Phase (degrees) the controller would have to predict ahead at freq_hz when margin < 0."""
    return 360.0 * freq_hz * max(0.0, -margin_s)


def format_budget(b, title="DELAY BUDGET"):
    lines = [title, "-" * len(title)]
    lines.append(f"  acoustic head start (ref mic -> speaker plane) : {b['head_start_s'] * 1e3:8.3f} ms")
    for name, v in b["items"]:
        lines.append(f"  - {name:<28}                  : {v * 1e3:8.3f} ms")
    lines.append(f"  total electronics / filter delay               : {b['total_s'] * 1e3:8.3f} ms")
    verdict = "reacts in time" if b["causal"] else "must PREDICT the noise ahead"
    lines.append(f"  margin                                         : {b['margin_s'] * 1e3:+8.3f} ms  ({verdict})")
    if not b["causal"]:
        for f in (100, 200, 300):
            lines.append(f"    prediction needed at {f:3d} Hz: {prediction_phase_deg(b['margin_s'], f):5.0f} deg ahead")
    return "\n".join(lines)
