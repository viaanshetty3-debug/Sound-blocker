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
    def test_speed_of_sound(self):
        self.assertAlmostEqual(tm.speed_of_sound(20.0), 343.23, places=1)
        self.assertAlmostEqual(tm.speed_of_sound(0.0), 331.3, delta=0.05)

    def test_filter_delays_match_hand_calculation(self):
        in_f, out_f = tm.PRESETS["original-rc"]
        self.assertAlmostEqual(in_f.dc_delay_s * 1e6, 990.0, places=6)     # 3RC, R=1.5k, C=220n
        self.assertAlmostEqual(out_f.dc_delay_s * 1e6, 330.0, places=6)    # RC
        b = tm.PRESETS["butterworth-1k"][0]
        self.assertAlmostEqual(b.dc_delay_s * 1e6, 225.079, places=2)      # sqrt(2)/(2 pi 1000)

    def test_sallen_key_with_purchased_parts(self):
        sk = tm.PRESETS["sk-11k-20n-10n"][0]
        self.assertAlmostEqual(sk.dc_delay_s * 1e6, 220.0, places=6)         # C2 (R1 + R2) = 10 nF x 22 k
        self.assertAlmostEqual(sk.fc_hz, 1023.09, places=2)                  # Q = 1/sqrt(2): same as Butterworth
        self.assertAlmostEqual(sk.gain_db(sk.fc_hz), -3.0103, places=3)
        self.assertAlmostEqual(sk.gain_db(2000.0), -11.932, places=3)        # at the 2 kHz Nyquist frequency (fs = 4 kHz)
        bw = tm.butterworth2(1023.0867229)
        for f in (100.0, 500.0, 1500.0, 3000.0):
            self.assertAlmostEqual(sk.gain_db(f), bw.gain_db(f), places=3)

    def test_ladder_minus_3db_frequency(self):
        lad = tm.PRESETS["original-rc"][0]
        self.assertAlmostEqual(lad.fc_hz, 180.49, places=2)
        self.assertAlmostEqual(lad.gain_db(lad.fc_hz), -3.0103, places=3)
        rc = tm.PRESETS["rc1-3k3-10n"][0]
        self.assertAlmostEqual(rc.gain_db(rc.fc_hz), -3.0103, places=3)
        bw = tm.PRESETS["butterworth-1k"][0]
        self.assertAlmostEqual(bw.gain_db(1000.0), -3.0103, places=3)

    def test_group_delay_is_largest_at_dc(self):
        """Numerically check that the DC delay is the maximum for the 1-pole RC (phase = -atan(w tau))."""
        import math
        tau = 33e-6
        def gd(f):
            w = 2 * math.pi * f
            return tau / (1 + (w * tau) ** 2)
        self.assertGreater(gd(0.0), gd(300.0))
        self.assertAlmostEqual(gd(0.0), tau)

    def test_band_residual_matches_numerical_integration(self):
        import math
        f1, f2, tau, g = 70.0, 300.0, 425.3e-6, 0.9
        n = 20000
        acc = 0.0
        for i in range(n):
            f = f1 + (f2 - f1) * (i + 0.5) / n
            acc += abs(1 - g * complex(math.cos(2 * math.pi * f * tau), -math.sin(2 * math.pi * f * tau))) ** 2
        self.assertAlmostEqual(tm.band_residual_db(f1, f2, tau, g), 10 * math.log10(acc / n), places=4)
        self.assertLess(tm.band_residual_db(f1, f2, 0.0, 1.0), -250)       # perfect: zero by construction

    def test_unknown_terms_are_never_guessed(self):
        in_f, out_f = tm.PRESETS["butterworth-1k"]
        b = tm.compute_budget(in_filter=in_f, out_filter=out_f)
        self.assertFalse(b["complete"])
        self.assertIsNone(b["margin_s"])
        self.assertIsNone(b["d_min_m"])
        self.assertIsNone(b["causal"])                                      # undetermined
        self.assertEqual(len(b["missing"]), 3)
        self.assertAlmostEqual(b["margin_upper_bound_s"] * 1e6, 444.6, delta=0.1)

    def test_negative_bound_is_definite(self):
        in_f, out_f = tm.PRESETS["original-rc"]
        b = tm.compute_budget(in_filter=in_f, out_filter=out_f)
        self.assertIs(b["causal"], False)                                   # unknown terms can only add delay
        self.assertAlmostEqual(b["margin_upper_bound_s"] * 1e6, -425.3, delta=0.1)
        self.assertAlmostEqual(b["d_min_lower_bound_m"] * 100, 49.60, delta=0.01)

    def test_complete_budget_gives_exact_numbers(self):
        in_f, out_f = tm.PRESETS["butterworth-1k"]
        b = tm.compute_budget(in_filter=in_f, out_filter=out_f, isr_s=12e-6, dac_settle_s=5e-6, amp_s=5e-6)
        self.assertTrue(b["complete"])
        self.assertIs(b["causal"], True)
        total = 2 * in_f.dc_delay_s + 125e-6 + 22e-6
        self.assertAlmostEqual(b["known_total_s"], total, places=12)
        self.assertAlmostEqual(b["d_min_m"], b["c_m_s"] * total, places=12)
        self.assertAlmostEqual(b["margin_s"], b["head_start_s"] - total, places=12)

    def test_alias_report_warns_when_cutoff_above_nyquist(self):
        bw = tm.PRESETS["butterworth-1k"][0]
        self.assertTrue(any("WARNING" in l for l in tm.anti_alias_report(bw, 1500.0)))
        self.assertFalse(any("WARNING" in l for l in tm.anti_alias_report(bw, 4000.0)))


class ToolTests(unittest.TestCase):
    def test_parse_kv(self):
        tag, kv = tt.parse_kv("@TIMING state=ACTIVE n=5 exec_us_avg=1.667")
        self.assertEqual(tag, "TIMING")
        self.assertEqual(kv["state"], "ACTIVE")
        self.assertEqual(kv["n"], 5.0)

    def test_budget_exit_codes(self):
        code, out = run(["budget", "--preset", "original-rc"])
        self.assertEqual(code, 2)
        self.assertIn("at least", out)
        code, out = run(["budget", "--preset", "butterworth-1k"])
        self.assertEqual(code, 3)                                           # undetermined, not a pass
        self.assertIn("NOT SUPPLIED", out)
        self.assertIn("undetermined", out)
        code, out = run(["budget", "--preset", "butterworth-1k", "--isr-us", "12",
                         "--dac-settle-us", "5", "--amp-us", "5"])
        self.assertEqual(code, 0, out)
        self.assertIn("all terms supplied", out)
        self.assertNotIn("NOT SUPPLIED:", out)

    def test_json_marks_unsupplied(self):
        import json
        code, out = run(["budget", "--preset", "rc1-3k3-10n", "--fs", "48000", "--json"])
        data = json.loads(out)
        status = {t["name"]: t["status"] for t in data["terms"]}
        self.assertEqual(status["ISR (ADC + maths + SPI write)"], "NOT SUPPLIED")
        self.assertIsNone(data["margin_s"])

    def test_measure_good_isr_but_unknown_terms_is_undetermined(self):
        dev = FakeDevice([ACTIVE_OK])
        code, out = run(["measure", "--port", "x", "--seconds", "0", "--preset", "butterworth-1k"], dev)
        self.assertEqual(code, 3, out)
        self.assertIn("worst case 12.000 us", out)
        self.assertEqual(dev.sent, ["?timing", "?timing"])

    def test_measure_with_all_terms_passes(self):
        dev = FakeDevice([ACTIVE_OK])
        code, out = run(["measure", "--port", "x", "--seconds", "0", "--preset", "butterworth-1k",
                         "--dac-settle-us", "5", "--amp-us", "5"], dev)
        self.assertEqual(code, 0, out)
        self.assertIn("measured, worst case", out)

    def test_measure_flags_overruns(self):
        dev = FakeDevice([ACTIVE_OVERRUN])
        code, out = run(["measure", "--port", "x", "--seconds", "0", "--preset", "butterworth-1k",
                         "--dac-settle-us", "5", "--amp-us", "5"], dev)
        self.assertEqual(code, 2)
        self.assertIn("samples are being lost", out)

    def test_measure_with_original_filters_fails(self):
        dev = FakeDevice([ACTIVE_OK])
        code, out = run(["measure", "--port", "x", "--seconds", "0", "--preset", "original-rc"], dev)
        self.assertEqual(code, 2)
        self.assertIn("at least", out)

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
