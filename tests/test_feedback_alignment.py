#!/usr/bin/env python3
"""
Host-side check of the firmware's index bookkeeping (no hardware needed).

Mirrors window_anc_fxlms.ino exactly where it matters:
  * calibration: read mics, THEN write v(n) and push it into y_buf (k=0 = current output),
    then NLMS-update F_hat against the regressor [v(n), v(n-1), ...]
  * active:      read mics, compute fb_est from y_buf BEFORE pushing the new output
    (so y_buf[0] is the output from ONE tick ago)

The plant is a pure delayed feedback path: a mic reading at tick n contains
f[k] * (output written k ticks earlier), with f[0] = 0 (a reading cannot depend on the
sample written after it).

Run:  python3 tests/test_feedback_alignment.py
"""
import numpy as np

S_TAPS = 64
rng = np.random.default_rng(1)
f_true = np.zeros(S_TAPS)
f_true[4], f_true[5], f_true[6] = 0.16, 0.09, -0.06      # same shape as the simulators' F(z)


def calibrate(steps=12000, mu=0.08):
    F_hat = np.zeros(S_TAPS)
    out_hist = np.zeros(S_TAPS + 1)                          # out_hist[k] = output k ticks ago (before this tick's write)
    y_buf = np.zeros(S_TAPS)
    for _ in range(steps):
        x_n = np.dot(f_true, out_hist[:S_TAPS])              # reading at tick n (before write)
        v = rng.uniform(-0.18, 0.18)
        out_hist = np.concatenate(([v], out_hist[:-1]))      # write v(n)
        y_buf = np.concatenate(([v], y_buf[:-1]))            # push (firmware pushes before the update)
        f_hat = np.dot(F_hat, y_buf)
        p = np.dot(y_buf, y_buf) + 1e-4
        F_hat += mu * (x_n - f_hat) / p * y_buf
    return F_hat


def residual_feedback_db(F_hat, shifted, steps=4000):
    """RMS of what is left of the speaker's feedback in the reference signal after neutralisation."""
    out_hist = np.zeros(S_TAPS + 1)
    y_buf = np.zeros(S_TAPS)
    num = den = 0.0
    for _ in range(steps):
        fb_true = np.dot(f_true, out_hist[:S_TAPS])
        if shifted:
            fb_est = np.dot(F_hat[1:], y_buf[:-1])           # pairing F_hat[j+1] <-> y_buf[j]
        else:
            fb_est = np.dot(F_hat, y_buf)                    # original firmware pairing
        num += (fb_true - fb_est) ** 2
        den += fb_true ** 2
        y = rng.uniform(-0.5, 0.5)                           # arbitrary anti-noise output
        out_hist = np.concatenate(([y], out_hist[:-1]))
        y_buf = np.concatenate(([y], y_buf[:-1]))            # pushed AFTER fb_est, as in the ISR
    return 10 * np.log10(num / den)


if __name__ == "__main__":
    F_hat = calibrate()
    old = residual_feedback_db(F_hat, shifted=False)
    new = residual_feedback_db(F_hat, shifted=True)
    print(f"feedback left in reference after neutralisation: original pairing {old:6.1f} dB, "
          f"fixed pairing {new:6.1f} dB")
    assert new < -25.0, new
    assert old > new + 15.0, (old, new)
    print("PASS: the one-sample-shifted pairing removes the feedback; the original pairing does not.")
