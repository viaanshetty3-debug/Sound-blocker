#!/usr/bin/env python3
"""
virtual_teensy.py - run the real firmware (window_anc_fxlms.ino) on this computer, no board needed.

The firmware is compiled unmodified against small host stubs of the Teensy core
(tools/virtual_teensy/*.h) and a harness (tools/virtual_teensy/harness.cpp) that runs the
4,000 Hz ISR in wall-clock time and connects the microphones and DAC to a toy plant.

REAL:      the firmware's code, state machine and serial protocol.
NOT REAL:  ISR timing (the cycle counter reports an ASSUMED time, --isr-us, default 250 us =
           the most the ISR may take without losing a sample) and the acoustics (two tones +
           noise through pure delays). Every command reply is preceded by an "@VIRTUAL" line,
           and timing_tool.py marks the ISR term as ASSUMED, never as measured.

Usage
  python3 tools/timing_tool.py measure --port virtual          # easiest: the tool starts it
  python3 tools/virtual_teensy.py --pty                        # serial port for other programs
  python3 tools/virtual_teensy.py                              # talk to it on stdin/stdout

Needs a C++ compiler (clang++ or g++; on a Mac: xcode-select --install). The build is cached
in tools/virtual_teensy/build/ and redone when a source file changes.
"""
import argparse
import os
import select
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
STUB_DIR = os.path.join(HERE, "virtual_teensy")
FIRMWARE = os.path.join(ROOT, "firmware", "window_anc_fxlms", "window_anc_fxlms.ino")
BUILD_DIR = os.path.join(STUB_DIR, "build")
BINARY = os.path.join(BUILD_DIR, "virtual_teensy")
DEFAULT_ISR_US = 250.0


def sources():
    names = ["harness.cpp", "Arduino.h", "SPI.h", "IntervalTimer.h"]
    return [FIRMWARE] + [os.path.join(STUB_DIR, n) for n in names]


def build(force=False):
    """Compile the firmware + harness if needed. Returns the binary path."""
    if not force and os.path.exists(BINARY):
        built = os.path.getmtime(BINARY)
        if all(os.path.getmtime(s) <= built for s in sources()):
            return BINARY
    cxx = os.environ.get("CXX") or shutil.which("clang++") or shutil.which("g++")
    if not cxx:
        raise RuntimeError("no C++ compiler found (on a Mac run: xcode-select --install)")
    os.makedirs(BUILD_DIR, exist_ok=True)
    cmd = [cxx, "-std=c++17", "-O2", "-I", STUB_DIR, "-x", "c++", FIRMWARE, "-x", "none",
           os.path.join(STUB_DIR, "harness.cpp"), "-o", BINARY]
    res = subprocess.run(cmd, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError("virtual Teensy build failed:\n" + res.stderr)
    return BINARY


class VirtualDevice:
    """Serial-like object (write(bytes), readline() -> bytes) talking to the virtual Teensy."""

    def __init__(self, isr_us=DEFAULT_ISR_US, timeout=0.2):
        self.timeout = timeout
        self.proc = subprocess.Popen([build(), "--isr-us", repr(float(isr_us))],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self.buf = b""

    def write(self, data):
        self.proc.stdin.write(data)
        self.proc.stdin.flush()

    def readline(self):
        fd = self.proc.stdout.fileno()
        while b"\n" not in self.buf:
            ready, _, _ = select.select([fd], [], [], self.timeout)
            if not ready:
                return b""
            chunk = os.read(fd, 4096)
            if not chunk:
                break
            self.buf += chunk
        line, sep, rest = self.buf.partition(b"\n")
        self.buf = rest
        return line + sep

    def close(self):
        if self.proc.poll() is None:
            self.proc.stdin.close()
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.proc.kill()

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass


def main(argv=None):
    ap = argparse.ArgumentParser(description="Run window_anc_fxlms.ino on this computer.")
    ap.add_argument("--isr-us", type=float, default=DEFAULT_ISR_US,
                    help=f"ASSUMED ISR execution time reported by the cycle counter (default {DEFAULT_ISR_US:g})")
    ap.add_argument("--pty", action="store_true", help="create a serial port (pseudo-terminal) and print its path")
    ap.add_argument("--build-only", action="store_true")
    args = ap.parse_args(argv)
    binary = build()
    if args.build_only:
        print(binary)
        return 0
    cmd = [binary, "--isr-us", repr(args.isr_us)] + (["--pty"] if args.pty else [])
    if args.pty:
        print("Virtual Teensy running. Use the port printed below, e.g.\n"
              "  python3 tools/timing_tool.py params --port <PORT>\nCtrl-C to stop.", file=sys.stderr)
    try:
        return subprocess.call(cmd)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
