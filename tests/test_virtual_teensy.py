#!/usr/bin/env python3
"""
Runs the real firmware (window_anc_fxlms.ino) on this computer through tools/virtual_teensy.py
and drives it with tools/timing_tool.py, so the serial parser tested here is the firmware's own,
not a copy. Skipped when no C++ compiler is installed. Takes about 10 s (the virtual board runs
in real time and calibrates for 3 s).

Run:  python3 -m unittest tests/test_virtual_teensy.py
"""
import io
import os
import shutil
import sys
import time
import unittest
from contextlib import redirect_stdout

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
import timing_model as tm      # noqa: E402
import timing_tool as tt       # noqa: E402
import virtual_teensy as vt    # noqa: E402

HAVE_CXX = bool(os.environ.get("CXX") or shutil.which("clang++") or shutil.which("g++"))


def run(argv):
    buf = io.StringIO()
    with redirect_stdout(buf):
        try:
            code = tt.main(argv)
        except SystemExit as e:
            code = e.code
    return code, buf.getvalue()


class AssumedTermTests(unittest.TestCase):
    def test_assumed_term_never_passes(self):
        b = tm.compute_budget(in_filter_s=0.0, out_filter_s=0.0, isr_s=10e-6, isr_status="ASSUMED (x)",
                              dac_settle_s=0.0, amp_s=0.0)
        self.assertTrue(b["complete"])
        self.assertIsNone(b["causal"])
        self.assertIn("(if assumed true)", tm.format_budget(b))

    def test_negative_without_assumed_term_is_still_definite(self):
        flt = tm.PRESETS["original-rc"]
        b = tm.compute_budget(in_filter=flt[0], out_filter=flt[1], isr_s=10e-6, isr_status="ASSUMED (x)")
        self.assertIs(b["causal"], False)

    def test_negative_only_because_of_assumed_term_is_undetermined(self):
        b = tm.compute_budget(in_filter_s=0.0, out_filter_s=0.0, isr_s=5e-3, isr_status="ASSUMED (x)")
        self.assertLess(b["margin_upper_bound_s"], 0)
        self.assertIsNone(b["causal"])


@unittest.skipUnless(HAVE_CXX, "no C++ compiler")
class VirtualTeensyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        vt.build()

    def test_commands_reach_the_real_parser(self):
        link = tt.Link(vt.VirtualDevice())
        try:
            tag, kv, _ = link.transact("?params")[-1]
            self.assertEqual((tag, kv["state"], kv["mu"]), ("PARAMS", "CALIBRATING", 0.015))
            self.assertIsNotNone(link.virtual)
            self.assertEqual(link.transact("set mu 0.5")[-1][0], "ERR")          # firmware range check
            self.assertEqual(link.transact("set mu 0.02")[-1][0], "OK")
            self.assertEqual(link.transact("set anc off")[-1][2], "@ERR busy calibrating")
            self.assertEqual(link.transact("bogus")[-1][0], "ERR")
            # Regression: 'c' used to be taken as the recalibrate key, leaving "alibrate".
            self.assertEqual(link.transact("calibrate")[-1][2], "@OK calibrating")
        finally:
            link.dev.close()

    def test_single_key_toggle_without_newline(self):
        dev = vt.VirtualDevice()
        try:
            time.sleep(3.3)                                   # calibration lasts 3 s
            dev.write(b"t")
            deadline, seen = time.monotonic() + 2.0, ""
            while time.monotonic() < deadline and "BYPASS (Speaker Muted)" not in seen:
                seen += dev.readline().decode()
            self.assertIn("[ANC] Mode -> BYPASS (Speaker Muted)", seen)
        finally:
            dev.close()

    def test_measure_on_virtual_board_is_never_a_pass(self):
        code, out = run(["measure", "--port", "virtual", "--seconds", "0.5",
                         "--dac-settle-us", "5", "--amp-us", "5", "--virtual-isr-us", "12"])
        self.assertEqual(code, 3, out)
        self.assertIn("VIRTUAL TEENSY", out)
        self.assertIn("worst case 12.000 us", out)
        self.assertIn("[ASSUMED (virtual Teensy)]", out)
        self.assertNotIn("measured, worst case", out)
        self.assertIn("rests on ASSUMED values", out)

    def test_calibrate_command_succeeds(self):
        code, out = run(["calibrate", "--port", "virtual"])
        self.assertEqual(code, 0, out)


if __name__ == "__main__":
    unittest.main(verbosity=2)
