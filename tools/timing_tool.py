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

`measure` exits with status 0 if the ISR never overran and the margin is >= 0, 2 otherwise.
Needs `pip install pyserial` for the serial commands (not for `budget`).

What `measure` can NOT tell you: the delay of your analog filters, amplifier and speaker, and the
real acoustic path. Those are inputs (--preset / --in-filter-ms / --out-filter-ms / --extra-ms);
measure them with an oscilloscope or a loopback test.
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
def filter_delays(args):
    if args.in_filter_ms is not None or args.out_filter_ms is not None:
        return (args.in_filter_ms or 0.0) * 1e-3, (args.out_filter_ms or 0.0) * 1e-3, "custom filter delays"
    d_in, d_out, desc = tm.PRESETS[args.preset]
    return d_in, d_out, f"preset {args.preset}: {desc}"


def budget_from_args(args, isr_exec_s):
    d_in, d_out, desc = filter_delays(args)
    b = tm.compute_budget(
        ref_to_spk_m=args.ref_spk_cm / 100.0, spk_to_err_m=args.spk_err_cm / 100.0,
        ref_to_err_m=None if args.ref_err_cm is None else args.ref_err_cm / 100.0,
        in_filter_s=d_in, out_filter_s=d_out, fs_hz=args.fs,
        isr_exec_s=isr_exec_s, extra_s=args.extra_ms * 1e-3)
    return b, desc


def cmd_budget(args):
    b, desc = budget_from_args(args, args.isr_us * 1e-6)
    if args.json:
        print(json.dumps({k: v for k, v in b.items() if k != "items"} | {"items": dict(b["items"])}, indent=2))
    else:
        print(f"filters: {desc}")
        print(f"ISR execution: {args.isr_us:.1f} us ({'assumed - run `measure` for a real value' if args.isr_us == 0 else 'user supplied'})")
        print(tm.format_budget(b))
    return 0 if b["causal"] else 2


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
    print(f"  execution  avg {exec_avg:8.3f} us   worst {exec_max:8.3f} us   ({100 * exec_max / period:.1f}% of the {period:.0f} us period)")
    ok = True
    if "interval_us_min" in t:
        jitter = max(t["interval_us_max"] - period, period - t["interval_us_min"])
        print(f"  tick interval  min {t['interval_us_min']:8.3f}  max {t['interval_us_max']:8.3f} us   (jitter {jitter:.3f} us)")
    if t.get("overruns", 0) > 0:
        print(f"  !! {int(t['overruns'])} ISR run(s) took longer than one sample period: samples are being lost")
        ok = False
    if exec_max > 0.5 * period:
        print("  !! worst-case execution is over half the period; little room for anything else")

    # The worst-case total step time is an upper bound for sample->DAC latency (the DAC is written
    # before the weight update), so using it keeps the verdict on the safe side.
    b, desc = budget_from_args(args, exec_max * 1e-6)
    print(f"\nfilters: {desc}")
    print(tm.format_budget(b, "DELAY BUDGET with measured ISR time"))
    print("\nnot measured here: analog filter, amplifier and woofer delay; the real acoustic path.")
    return 0 if (ok and b["causal"]) else 2


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
        p.add_argument("--extra-ms", type=float, default=0.0, help="any other measured delay (amp, DAC, ...)")
        p.add_argument("--ref-spk-cm", type=float, default=35.0)
        p.add_argument("--spk-err-cm", type=float, default=25.0)
        p.add_argument("--ref-err-cm", type=float, help="direct ref->error distance if not collinear")
        p.add_argument("--fs", type=float, default=4000.0, help="controller sample rate in Hz")

    def port(p):
        p.add_argument("--port", required=True)
        p.add_argument("--baud", type=int, default=115200)

    b = sub.add_parser("budget", help="offline delay budget")
    geometry(b)
    b.add_argument("--isr-us", type=float, default=0.0, help="ISR execution time (default 0 = not yet measured)")
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
