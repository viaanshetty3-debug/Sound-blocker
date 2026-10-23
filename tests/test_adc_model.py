#!/usr/bin/env python3
"""
Checks the HIL simulator's ADC noise model against a measurement of the quantizer itself.

adc_enob() is a closed form; here the same quantizer is driven with a near-full-scale sine
and the ENOB is measured from the actual error (output - input), so the formula is not
checked against itself.

Run:  python3 -m unittest tests/test_adc_model.py
"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "simulator"))
import simulate_realistic_hil as hil  # noqa: E402


def measured_enob(noise_lsb, n=400_000, seed=3):
    rng = np.random.default_rng(seed)
    t = np.arange(n)
    x = 0.98 * np.sin(2.0 * np.pi * 0.0123457 * t)            # non-harmonic frequency, no clipping
    err = hil.quantize_12bit_adc(x, rng, noise_lsb) - x
    lsb = 1.0 / 2048.0
    return 12.0 - np.log2(np.std(err) / (lsb / np.sqrt(12.0)))


class AdcModelTest(unittest.TestCase):
    def test_default_noise_is_about_ten_bits(self):
        self.assertAlmostEqual(hil.adc_enob(hil.ADC_NOISE_LSB), 9.904, places=3)

    def test_inverse(self):
        for enob in (12.0, 11.0, 10.0, 9.0, 8.0):
            self.assertAlmostEqual(hil.adc_enob(hil.adc_noise_lsb_for_enob(enob)), enob, places=12)
        with self.assertRaises(ValueError):
            hil.adc_noise_lsb_for_enob(12.5)

    def test_formula_matches_measured_quantizer(self):
        for noise_lsb in (0.0, 0.5, hil.ADC_NOISE_LSB, 4.6):
            self.assertAlmostEqual(measured_enob(noise_lsb), hil.adc_enob(noise_lsb), delta=0.02)


if __name__ == "__main__":
    unittest.main()
