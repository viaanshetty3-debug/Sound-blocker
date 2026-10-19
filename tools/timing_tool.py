#!/usr/bin/env python3
"""
timing_tool.py - plan and check the timing of the Teensy ANC controller from a computer.

Python does NOT run the control loop (an OS-scheduled process has milliseconds of jitter; the
budget is about 1 ms). The Teensy's timer ISR does that. This tool does the parts a PC is good at:

  budget    Offline delay budget from geometry + filter choice (no hardware needed).
  measure   Ask the firmware how long its ISR really takes and how steady the 250 us tick is,
            then plug the measured worst case into the same budget and print a verdict.
  params    Show the firmware's current settings.
  set       Change settings at runtime: anc on|off, mu <v>, leak <v>.
  calibrate Start the 3 s secondary-path calibration.

Examples
  python3 tools/timing_tool.py budget --preset original-rc
  python3 tools/timing_tool.py budget --preset butterworth-1k --isr-us 12
  python3 tools/timing_tool.py measure --port /dev/cu.usbmodem1234 --preset butterworth-1k
  python3 tools/timing_tool.py set --port /dev/cu.usbmodem1234 mu 0.012

Strict bookkeeping (see tools/timing_model.py): a delay term that was not supplied is never
guessed. Until the ISR time, the DAC settling time and the amplifier delay are supplied, the
result is a BOUND (margin <= x, break-even distance >= y), not a single number.

Exit status of `budget` and `measure`:
  0  every term supplied, margin >= 0, no ISR overrun
  2  margin is negative (already true with the missing terms at zero), or the ISR overran
  3  undetermined: terms are still missing and the bound does not rule the design out

Needs `pip install pyserial` for the serial commands (not for `budget`).

What `measure` does NOT know: the delay of your analog filters, DAC settling, amplifier and
speaker, and the real acoustic path. Enter those with --preset / --in-filter-ms /
--out-filter-ms / --dac-settle-us / --amp-us / --extra-ms (oscilloscope, datasheet).
"""
import argparse
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import timing_model as tm  # noqa: E402

PERIOD_US_DEFAULT = 250.0


# ----------------------------------------------------------------------------
# Serial protocol (see the "Serial interface" comment in window_anc_fxlms.ino)
# ----------------------------------------------------------------------------
def parse_kv(line):
    """'@TIMING state=ACTIVE n=5 exec_us_avg=1.2' -> ('TIMING', {'state': 'ACTIVE', 'n': 5.0, ...})."""
    parts = line.strip().split()
    tag = parts[0].lstrip("@")
    kv = {}
    for p in parts[1:]:
        if "=" in p:
            k, v = p.split("=", 1)
            try:
                kv[k] = float(v)
            except ValueError:
                kv[k] = v
    return tag, kv


class Link:
    """Thin wrapper over anything with write(bytes) and readline() -> bytes (pyserial or a fake)."""

    def __init__(self, dev):
        self.dev = dev

    def send(self, text):
        self.dev.write((text.strip() + "\n").encode("ascii"))

    def transact(self, text, timeout_s=3.0, until=("END", "OK", "ERR", "PARAMS")):
        """Send one command; collect '@' reply lines until a terminator tag. Other lines are ignored."""
        self.send(text)
        replies = []
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            raw = self.dev.readline()
            if not raw:
                continue
            line = raw.decode("ascii", "replace").strip()
            if not line.startswith("@"):
                continue                                  # human-readable telemetry: skip
            tag, kv = parse_kv(line)
            replies.append((tag, kv, line))
            if tag in until:
                return replies
        raise TimeoutError(f"no reply to {text!r} within {timeout_s} s (is the firmware running?)")


def open_serial(port, baud=115200, timeout=0.2):
    try:
        import serial  # pyserial
    except ImportError:
        sys.exit("pyserial is required for serial commands: pip install pyserial")
    return Link(serial.Serial(port, baud, timeout=timeout))


# ----------------------------------------------------------------------------
# Commands
# ----------------------------------------------------------------------------
def make_budget(args, isr_s=None, isr_status="supplied"):
    """Build the budget from CLI args. Returns (budget, in_filter, out_filter, description)."""
    in_f, out_f = tm.PRESETS[args.preset]
    in_s = out_s = None
    desc = f"preset {args.preset}"
    if args.in_filter_ms is not None or args.out_filter_ms is not None:
        in_s = (args.in_filter_ms or 0.0) * 1e-3
        out_s = (args.out_filter_ms or 0.0) * 1e-3
        in_f = out_f = None
        desc = "custom measured filter delays"
    b = tm.compute_budget(
        ref_to_spk_m=args.ref_spk_cm / 100.0, spk_to_err_m=args.spk_err_cm / 100.0,
        ref_to_err_m=None if args.ref_err_cm is None else args.ref_err_cm / 100.0,
        in_filter=in_f, out_filter=out_f, in_filter_s=in_s, out_filter_s=out_s,
        fs_hz=args.fs, isr_s=isr_s, isr_status=isr_status,
        dac_settle_s=None if args.dac_settle_us is None else args.dac_settle_us * 1e-6,
        amp_s=None if args.amp_us is None else args.amp_us * 1e-6,
        wire_m=args.wire_m, extra_s=args.extra_ms * 1e-3, temp_c=args.temp_c)
    return b, in_f, out_f, desc


def exit_code(b, overran=False):
    if overran or b["causal"] is False:
        return 2
    return 0 if b["causal"] else 3


def print_filter_report(in_f, out_f, fs, band_max=300.0):
    for label, flt in (("input", in_f), ("output", out_f)):
        if flt is None:
            continue
        print(f"\n{label} filter: {flt.description}")
        print(f"  DC group delay (upper bound over the band): {flt.dc_delay_s * 1e6:.3f} us")
        for line in tm.anti_alias_report(flt, fs, band_max):
            print(line)


def cmd_budget(args):
    isr_s = None if args.isr_us is None else args.isr_us * 1e-6
    b, in_f, out_f, desc = make_budget(args, isr_s)
    if args.json:
        out = dict(b)
        out["terms"] = [{"name": n, "seconds": v, "status": st} for n, v, st in b["terms"]]
        print(json.dumps(out, indent=2))
    else:
        print(f"filters: {desc}")
        print(tm.format_budget(b))
        print_filter_report(in_f, out_f, args.fs)
    return exit_code(b)


def pick_state(timings):
    for want in ("ACTIVE", "BYPASS", "CALIBRATING"):
        for t in timings:
            if t.get("state") == want and t.get("n", 0) > 0:
                return t
    return None


def cmd_measure(args, link=None):
    link = link or open_serial(args.port, args.baud)
    link.transact("?timing")                               # discard stats gathered so far
    print(f"collecting ISR timing for {args.seconds:.1f} s ...")
    time.sleep(args.seconds)
    replies = link.transact("?timing")
    timings = [kv for tag, kv, _ in replies if tag == "TIMING"]
    t = pick_state(timings)
    if t is None:
        print("no ISR samples were recorded (is the timer running?)")
        return 2

    period = t.get("period_us", PERIOD_US_DEFAULT)
    exec_max, exec_avg = t["exec_us_max"], t["exec_us_avg"]
    print(f"\nISR timing, state {t['state']}, {int(t['n'])} samples")
    print(f"  execution  average {exec_avg:.3f} us   worst case {exec_max:.3f} us   (worst case = {100 * exec_max / period:.1f}% of the {period:.0f} us period)")
    overran = t.get("overruns", 0) > 0
    if "interval_us_min" in t:
        jitter = max(t["interval_us_max"] - period, period - t["interval_us_min"])
        print(f"  tick interval  min {t['interval_us_min']:.3f} us  max {t['interval_us_max']:.3f} us   (largest deviation from {period:.0f} us: {jitter:.3f} us)")
    if overran:
        print(f"  !! {int(t['overruns'])} ISR run(s) took longer than one sample period: samples are being lost")

    # exec_max covers the whole control step. The DAC is written before the weight update, so the
    # true sample->DAC time is at most this: the figure is an upper bound on that delay.
    b, in_f, out_f, desc = make_budget(args, exec_max * 1e-6, isr_status="measured, worst case")
    print(f"\nfilters: {desc}")
    print(tm.format_budget(b, "DELAY BUDGET with measured ISR time"))
    print_filter_report(in_f, out_f, args.fs)
    return exit_code(b, overran)


def cmd_params(args, link=None):
    link = link or open_serial(args.port, args.baud)
    for tag, kv, line in link.transact("?params"):
        if tag == "PARAMS":
            print(line[1:])
    return 0


def cmd_set(args, link=None):
    name, value = args.name, args.value
    if name == "anc":
        if value not in ("on", "off"):
            sys.exit("set anc needs 'on' or 'off'")
        text = f"set anc {value}"
    elif name == "mu":
        if not 0.0005 <= float(value) <= 0.1:
            sys.exit("mu must be between 0.0005 and 0.1")
        text = f"set mu {float(value):g}"
    elif name == "leak":
        if not 0.999 <= float(value) <= 1.0:
            sys.exit("leak must be between 0.999 and 1.0")
        text = f"set leak {float(value):.6f}"
    else:
        sys.exit("unknown setting")
    link = link or open_serial(args.port, args.baud)
    tag, _, line = link.transact(text)[-1]
    print(line[1:])
    return 0 if tag == "OK" else 2


def cmd_calibrate(args, link=None):
    link = link or open_serial(args.port, args.baud)
    tag, _, line = link.transact("calibrate")[-1]
    print(line[1:])
    return 0 if tag == "OK" else 2


def build_parser():
    ap = argparse.ArgumentParser(description="Plan and check ANC controller timing.")
    sub = ap.add_subparsers(dest="cmd", required=True)

    def geometry(p):
        p.add_argument("--preset", choices=sorted(tm.PRESETS), default="butterworth-1k")
        p.add_argument("--in-filter-ms", type=float, help="measured input filter delay (overrides preset)")
        p.add_argument("--out-filter-ms", type=float, help="measured output filter delay (overrides preset)")
        p.add_argument("--dac-settle-us", type=float, help="DAC output settling time (datasheet); unknown if omitted")
        p.add_argument("--amp-us", type=float, help="power amplifier delay (scope); unknown if omitted")
        p.add_argument("--wire-m", type=float, default=0.0, help="total signal wire length counted (default 0)")
        p.add_argument("--extra-ms", type=float, default=0.0, help="any other delay you have measured")
        p.add_argument("--temp-c", type=float, default=20.0, help="air temperature for the speed of sound")
        p.add_argument("--ref-spk-cm", type=float, default=35.0)
        p.add_argument("--spk-err-cm", type=float, default=25.0)
        p.add_argument("--ref-err-cm", type=float, help="direct ref->error distance if not collinear")
        p.add_argument("--fs", type=float, default=4000.0, help="controller sample rate in Hz")

    def port(p):
        p.add_argument("--port", required=True)
        p.add_argument("--baud", type=int, default=115200)

    b = sub.add_parser("budget", help="offline delay budget")
    geometry(b)
    b.add_argument("--isr-us", type=float, default=None, help="ISR time (ADC+maths+SPI); unknown if omitted")
    b.add_argument("--json", action="store_true")
    b.set_defaults(fn=cmd_budget)

    m = sub.add_parser("measure", help="measure ISR time on the board and judge the budget")
    geometry(m)
    port(m)
    m.add_argument("--seconds", type=float, default=5.0)
    m.set_defaults(fn=cmd_measure)

    p = sub.add_parser("params", help="show firmware settings")
    port(p)
    p.set_defaults(fn=cmd_params)

    s = sub.add_parser("set", help="change a firmware setting")
    port(s)
    s.add_argument("name", choices=["anc", "mu", "leak"])
    s.add_argument("value")
    s.set_defaults(fn=cmd_set)

    c = sub.add_parser("calibrate", help="start secondary-path calibration")
    port(c)
    c.set_defaults(fn=cmd_calibrate)
    return ap


def main(argv=None, link=None):
    args = build_parser().parse_args(argv)
    if args.cmd == "budget":
        return args.fn(args)
    return args.fn(args, link=link)


if __name__ == "__main__":
    sys.exit(main())
