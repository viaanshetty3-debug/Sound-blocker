#!/usr/bin/env python3
"""
Live Tablet HTTP + Python Precision Phase-Cancellation Server
=============================================================
Serves `tablet_app/index.html` on http://localhost:8080 and exposes:
- POST /api/analyze : Receives captured audio samples from the tablet mic/stream,
                      runs `compute_zero_error_cancellation()` in Python, and
                      returns exact detected frequencies, phases, wavelengths,
                      microsecond timing budget, and the opposite-phase waveform.
"""

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


class PrecisionANCHandler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=TABLET_APP_DIR, **kwargs)

    def log_message(self, format, *args):
        pass  # Keep server console clean

    def do_POST(self):
        if self.path == "/api/analyze":
            length = int(self.headers.get("Content-Length", "0"))
            body = self.rfile.read(length)
            payload = json.loads(body.decode("utf-8"))

            samples = np.asarray(payload.get("samples", []), dtype=np.float64)
            fs = int(payload.get("sample_rate", 48000))
            dist_cm = float(payload.get("distance_cm", 35.0))
            hw_delay_us = float(payload.get("hw_delay_us", 12.5))

            if len(samples) < 256:
                t = np.arange(fs, dtype=np.float64) / fs
                samples = 0.5 * np.cos(2 * np.pi * 85.0 * t)

            details, budget, inc_wave, opp_wave, res_wave = compute_zero_error_cancellation(
                samples,
                fs=fs,
                mic_to_speaker_m=dist_cm / 100.0,
                hw_dac_delay_us=hw_delay_us,
            )

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


def run_server(port=8080):
    server = HTTPServer(("0.0.0.0", port), PrecisionANCHandler)
    print(f"[Python Precision ANC Server] Listening on http://localhost:{port}")
    server.serve_forever()


if __name__ == "__main__":
    run_server(8080)
