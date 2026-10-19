#!/usr/bin/env python3
"""
Tests for tools/timing_model.py and tools/timing_tool.py. No hardware needed: a fake serial
device replays reply lines in exactly the format window_anc_fxlms.ino prints (checked by running
the firmware's own command parser on a host build). The numbers it returns are canned test data,
NOT measurements.

Run:  python3 -m unittest tests/test_timing_tool.py   (or  python3 tests/test_timing_tool.py)
"""
import io
import os
import sys
import unittest
from contextlib import redirect_stdout

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import timing_model as tm      # noqa: E402
import timing_tool as tt       # noqa: E402


class FakeDevice:
    """Mimics pyserial: write(bytes) records a command, readline() replays queued reply lines."""

    def __init__(self, timing_lines=None, params=True):
        self.sent = []
        self.queue = []
        self.timing_lines = timing_lines or []
        self.params = params
        self.timing_calls = 0

    def write(self, data):
        cmd = data.decode().strip()
        self.sent.append(cmd)
        q = self.queue
        q.append(b"[STATUS: ANC ON ] Outdoor RMS: 0.1000 | human readable noise the tool must skip\n")
        if cmd == "?timing":
            self.timing_calls += 1
            if self.timing_calls == 1:
                q.append(b"@END\n")                         # first call just clears the counters
            else:
                for l in self.timing_lines:
                    q.append((l + "\n").encode())
                q.append(b"@END\n")
        elif cmd == "?params":
            q.append(b"@PARAMS state=ACTIVE fs_hz=4000 w_taps=128 s_taps=64 mu=0.015000 leak=0.999950 cpu_hz=600000000\n")
        elif cmd == "calibrate":
            q.append(b"@OK calibrating\n")
        elif cmd.startswith("set anc"):
            q.append(b"@OK anc\n")
        elif cmd.startswith("set mu") or cmd.startswith("set leak"):
            q.append(b"@OK " + cmd.split()[1].encode() + b"\n")
        else:
            q.append(b"@ERR unknown command\n")

    def readline(self):
        return self.queue.pop(0) if self.queue else b""


ACTIVE_OK = ("@TIMING state=ACTIVE n=19998 exec_us_avg=6.000 exec_us_max=12.000 "
             "interval_us_min=249.800 interval_us_max=250.300 overruns=0 period_us=250")
ACTIVE_OVERRUN = ("@TIMING state=ACTIVE n=19000 exec_us_avg=240.000 exec_us_max=310.000 "
                  "interval_us_min=200.000 interval_us_max=320.000 overruns=7 period_us=250")


def run(argv, dev=None):
    buf = io.StringIO()
    with redirect_stdout(buf):
        try:
            code = tt.main(argv, link=tt.Link(dev) if dev else None)
        except SystemExit as e:
            code = e.code
    return code, buf.getvalue()


class ModelTests(unittest.TestCase):
    def test_original_rc_delay_matches_hand_calculation(self):
        d_in, d_out, _ = tm.PRESETS["original-rc"]
        self.assertAlmostEqual(d_in * 1e3, 0.99, places=3)     # 3RC
        self.assertAlmostEqual(d_out * 1e3, 0.33, places=3)    # RC

    def test_butterworth_delay(self):
        self.assertAlmostEqual(tm.butterworth2_delay_s(1000.0) * 1e3, 0.2251, places=3)

    def test_head_start_is_ref_to_speaker_when_collinear(self):
        self.assertAlmostEqual(tm.head_start_s(0.35, 0.25) * 1e3, 0.35 / 343.0 * 1e3, places=6)

    def test_original_circuit_has_negative_margin_butterworth_positive(self):
        d_in, d_out, _ = tm.PRESETS["original-rc"]
        bad = tm.compute_budget(in_filter_s=d_in, out_filter_s=d_out)
        self.assertFalse(bad["causal"])
        self.assertAlmostEqual(bad["margin_s"] * 1e3, -0.425, places=2)
        d_in, d_out, _ = tm.PRESETS["butterworth-1k"]
        good = tm.compute_budget(in_filter_s=d_in, out_filter_s=d_out)
        self.assertTrue(good["causal"])
        self.assertAlmostEqual(good["margin_s"] * 1e3, 0.445, places=2)


class ToolTests(unittest.TestCase):
    def test_parse_kv(self):
        tag, kv = tt.parse_kv("@TIMING state=ACTIVE n=5 exec_us_avg=1.667")
        self.assertEqual(tag, "TIMING")
        self.assertEqual(kv["state"], "ACTIVE")
        self.assertEqual(kv["n"], 5.0)

    def test_budget_command_exit_codes(self):
        code, out = run(["budget", "--preset", "original-rc"])
        self.assertEqual(code, 2)
        self.assertIn("must PREDICT", out)
        code, out = run(["budget", "--preset", "butterworth-1k", "--isr-us", "12"])
        self.assertEqual(code, 0)
        self.assertIn("reacts in time", out)

    def test_measure_good_isr(self):
        dev = FakeDevice([ACTIVE_OK])
        code, out = run(["measure", "--port", "x", "--seconds", "0", "--preset", "butterworth-1k"], dev)
        self.assertEqual(code, 0, out)
        self.assertIn("worst   12.000 us", out)
        self.assertEqual(dev.sent, ["?timing", "?timing"])

    def test_measure_flags_overruns(self):
        dev = FakeDevice([ACTIVE_OVERRUN])
        code, out = run(["measure", "--port", "x", "--seconds", "0", "--preset", "butterworth-1k"], dev)
        self.assertEqual(code, 2)
        self.assertIn("samples are being lost", out)

    def test_measure_with_original_filters_fails_budget_even_if_isr_is_fine(self):
        dev = FakeDevice([ACTIVE_OK])
        code, out = run(["measure", "--port", "x", "--seconds", "0", "--preset", "original-rc"], dev)
        self.assertEqual(code, 2)
        self.assertIn("must PREDICT", out)

    def test_measure_with_no_samples(self):
        dev = FakeDevice([])
        code, out = run(["measure", "--port", "x", "--seconds", "0"], dev)
        self.assertEqual(code, 2)

    def test_set_commands_and_validation(self):
        dev = FakeDevice()
        code, _ = run(["set", "--port", "x", "mu", "0.012"], dev)
        self.assertEqual((code, dev.sent[-1]), (0, "set mu 0.012"))
        code, _ = run(["set", "--port", "x", "anc", "off"], dev)
        self.assertEqual((code, dev.sent[-1]), (0, "set anc off"))
        n = len(dev.sent)
        code, _ = run(["set", "--port", "x", "mu", "5"], dev)         # rejected before sending
        self.assertNotEqual(code, 0)
        code, _ = run(["set", "--port", "x", "leak", "0.5"], dev)
        self.assertNotEqual(code, 0)
        self.assertEqual(len(dev.sent), n)

    def test_params_and_calibrate(self):
        dev = FakeDevice()
        code, out = run(["params", "--port", "x"], dev)
        self.assertEqual(code, 0)
        self.assertIn("mu=0.015000", out)
        code, _ = run(["calibrate", "--port", "x"], dev)
        self.assertEqual(code, 0)

    def test_timeout_when_device_silent(self):
        class Silent:
            def write(self, d): pass
            def readline(self): return b""
        link = tt.Link(Silent())
        with self.assertRaises(TimeoutError):
            link.transact("?params", timeout_s=0.2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
