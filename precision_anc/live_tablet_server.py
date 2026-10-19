#!/usr/bin/env python3
"""
Live Tablet HTTP + Python Phase/Timing Calculator Server
=============================================================
Serves `tablet_app/index.html` on http://localhost:8080 and exposes:
- POST /api/analyze : Receives captured audio samples from the tablet mic/stream,
                      runs `compute_zero_error_cancellation()` in Python, and
                      returns detected frequencies, phases, wavelengths, the timing
                      budget, and the waveforms (incoming, anti-noise, residual with
                      the gain/timing errors the client asks for).

Binds to 127.0.0.1 by default. To use it from a tablet on your LAN pass
`--host 0.0.0.0` (anyone on the network can then reach it; there is no auth).
"""

import argparse
import json
import os
import sys
from http.server import SimpleHTTPRequestHandler, HTTPServer
import numpy as np

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(THIS_DIR)
TABLET_APP_DIR = os.path.join(ROOT_DIR, "tablet_app")
sys.path.insert(0, THIS_DIR)

from precision_phase_canceller import compute_zero_error_cancellation


MAX_BODY_BYTES = 8 * 1024 * 1024   # 4096-sample frames are ~100 KB; refuse anything huge
MAX_SAMPLES = 2 * 48000


class PrecisionANCHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=TABLET_APP_DIR, **kwargs)

    def log_message(self, format, *args):
        pass  # Keep server console clean

    def do_POST(self):
        if self.path == "/api/analyze":
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= MAX_BODY_BYTES:
                    raise ValueError("bad Content-Length")
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                samples = np.asarray(payload.get("samples", []), dtype=np.float64)[:MAX_SAMPLES]
                fs = int(payload.get("sample_rate", 48000))
                dist_cm = float(payload.get("distance_cm", 35.0))
                hw_delay_us = float(payload.get("hw_delay_us", 12.5))
                gain_err = float(payload.get("gain_error", 0.05))
                delay_err_us = float(payload.get("delay_error_us", 50.0))
                if not (8000 <= fs <= 192000 and 1.0 <= dist_cm <= 500.0
                        and 0.0 <= hw_delay_us <= 5000.0 and -1.0 <= gain_err <= 1.0
                        and -5000.0 <= delay_err_us <= 5000.0 and np.all(np.isfinite(samples))):
                    raise ValueError("parameter out of range")
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                self.send_error(400, f"bad request: {exc}")
                return

            if len(samples) < 256:
                t = np.arange(fs, dtype=np.float64) / fs
                samples = 0.5 * np.cos(2 * np.pi * 85.0 * t)

            try:
                details, budget, inc_wave, opp_wave, res_wave = compute_zero_error_cancellation(
                    samples,
                    fs=fs,
                    mic_to_speaker_m=dist_cm / 100.0,
                    hw_dac_delay_us=hw_delay_us,
                    gain_error=gain_err,
                    delay_error_us=delay_err_us,
                )
            except ValueError as exc:  # e.g. causality violation (mic too close)
                self.send_error(422, str(exc)[:200])
                return

            # Return a 512-sample slice after warmup for live oscilloscope verification
            warmup = min(len(inc_wave) - 512, max(64, int((dist_cm / 100.0 / 344.63) * fs) + 40))
            warmup = max(0, warmup)
            resp_data = {
                "details": details,
                "budget": budget,
                "wave_slice": {
                    "incoming": [round(float(x), 6) for x in inc_wave[warmup : warmup + 400]],
                    "opposite": [round(float(x), 6) for x in opp_wave[warmup : warmup + 400]],
                    "residual": [round(float(x), 10) for x in res_wave[warmup : warmup + 400]],
                },
            }

            out_bytes = json.dumps(resp_data).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(out_bytes)))
            self.end_headers()
            self.wfile.write(out_bytes)
        else:
            self.send_error(404)


def run_server(host="127.0.0.1", port=8080):
    server = HTTPServer((host, port), PrecisionANCHandler)
    print(f"[ANC calculator server] Listening on http://{host}:{port}")
    server.serve_forever()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    args = ap.parse_args()
    run_server(args.host, args.port)
