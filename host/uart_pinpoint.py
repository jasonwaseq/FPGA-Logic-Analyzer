#!/usr/bin/env python3
"""
uart_pinpoint.py — low-level UART path diagnostics.

Designed to be used with the optional diagnostic FPGA image (`uart_diag_top`):
  - checks idle-line noise
  - checks for beacon bytes (0x55)
  - checks byte echo reliability
"""

from __future__ import annotations

import argparse
from pathlib import Path
import random
import time
from dataclasses import dataclass

import serial


@dataclass
class Stat:
    name: str
    ok: bool
    detail: str


def _read_window(ser: serial.Serial, seconds: float) -> bytes:
    deadline = time.monotonic() + seconds
    buf = bytearray()
    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)
        else:
            time.sleep(0.01)
    return bytes(buf)


def _idle_noise(ser: serial.Serial, seconds: float) -> Stat:
    ser.reset_input_buffer()
    data = _read_window(ser, seconds)
    if not data:
        return Stat("idle_noise", True, "no unsolicited bytes")
    sample = data[:64].hex(" ")
    return Stat("idle_noise", False, f"unsolicited_bytes={len(data)} sample={sample}")


def _beacon(ser: serial.Serial, seconds: float) -> Stat:
    ser.reset_input_buffer()
    data = _read_window(ser, seconds)
    n55 = data.count(0x55)
    ok = n55 >= 2
    return Stat("beacon_0x55", ok, f"bytes={len(data)} count_0x55={n55}")


def _echo(ser: serial.Serial, n: int) -> Stat:
    ser.reset_input_buffer()
    pats = bytes((random.randint(1, 255) for _ in range(n)))
    ser.write(pats)
    ser.flush()
    got = _read_window(ser, 1.5)
    # Keep only first n bytes as expected echoes
    got_n = got[:n]
    ok = got_n == pats
    return Stat(
        "echo",
        ok,
        f"sent={pats.hex(' ')} got={got_n.hex(' ')} extra={max(0, len(got)-n)}",
    )


def main() -> int:
    ap = argparse.ArgumentParser(description="Pinpoint UART link issues on hardware.")
    ap.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB1")
    ap.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    ap.add_argument("--timeout", type=float, default=0.05, help="Read timeout (default 50ms)")
    args = ap.parse_args()

    if not Path(args.port).exists():
        print(f"FAIL: port does not exist: {args.port}")
        return 2

    stats: list[Stat] = []
    with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
        stats.append(_idle_noise(ser, 0.5))
        stats.append(_beacon(ser, 1.0))
        stats.append(_echo(ser, 8))

    any_fail = False
    print("UART pinpoint report")
    print("===================")
    for s in stats:
        mark = "PASS" if s.ok else "FAIL"
        if not s.ok:
            any_fail = True
        print(f"[{mark}] {s.name}: {s.detail}")

    print("\nOverall:", "PASS" if not any_fail else "FAIL")
    return 0 if not any_fail else 1


if __name__ == "__main__":
    raise SystemExit(main())
