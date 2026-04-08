#!/usr/bin/env python3
"""
hw_validate.py — End-to-end hardware validation for FPGA Logic Analyzer.

Usage (from repo root):
    python -m host.hw_validate --port /dev/ttyUSB1

This script verifies:
  1) Optional synthesis/programming
  2) UART protocol responsiveness (PING/STATUS/RESET)
  3) Register write/read round-trip
  4) RAW capture
  5) RLE capture

It produces a concise pass/fail report and exits non-zero on failure.
"""

from __future__ import annotations

import argparse
import json
import glob
import subprocess
import sys
import time
import tempfile
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional

import serial
from serial.tools import list_ports

from . import la_protocol as proto
from .la_capture import CaptureConfig, run_capture
from .la_device import LogicAnalyzerDevice
from .la_export import export_vcd, export_csv


@dataclass
class CheckResult:
    name: str
    ok: bool
    detail: str
    elapsed_s: float


def _now() -> float:
    return time.monotonic()


def _run_cmd(cmd: list[str], cwd: Path) -> tuple[int, str]:
    p = subprocess.run(cmd, cwd=str(cwd), capture_output=True, text=True)
    out = (p.stdout or "") + (p.stderr or "")
    return p.returncode, out


def _detect_icebreaker_ports() -> list[str]:
    ports: list[str] = []
    for p in list_ports.comports():
        desc = (p.description or "").lower()
        hwid = (p.hwid or "").lower()
        if "icebreaker" in desc or "vid:pid=0403:6010" in hwid:
            ports.append(p.device)
    return sorted(set(ports))


def _port_exists(port: str) -> bool:
    return Path(port).exists()


def _list_local_serial_candidates() -> list[str]:
    cands = sorted(set(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")))
    return cands


def _uart_probe_once(port: str, baud: int, timeout_s: float) -> str:
    """
    Low-level probe: send a raw PING frame and collect immediate bytes.
    Returns a short human-readable diagnostic string.
    """
    ping = bytes([proto.MAGIC, proto.CMD_PING, 0x00, proto.CMD_PING])
    try:
        with serial.Serial(port=port, baudrate=baud, timeout=timeout_s) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()
            ser.write(ping)
            ser.flush()
            time.sleep(0.05)
            rx = ser.read(64)
            if rx:
                return f"rx_bytes={rx.hex(' ')}"
            return "rx_bytes=<none>"
    except Exception as e:  # pragma: no cover - diagnostic path
        return f"serial_probe_error={type(e).__name__}: {e}"


def _check_make(repo: Path, target: str) -> CheckResult:
    t0 = _now()
    rc, out = _run_cmd(["make", target], cwd=repo)
    ok = rc == 0
    detail = f"`make {target}` {'ok' if ok else 'failed'}"
    if not ok:
        tail = "\n".join(out.strip().splitlines()[-20:])
        detail += f"\n--- tail ---\n{tail}"
    return CheckResult(f"make_{target}", ok, detail, _now() - t0)


def _check_ping_status(port: str, baud: int, timeout: float, deep: bool) -> CheckResult:
    t0 = _now()
    try:
        with LogicAnalyzerDevice(port, baud=baud, timeout=timeout) as la:
            if not la.ping():
                probe = _uart_probe_once(port, baud, timeout) if deep else "no deep probe"
                return CheckResult(
                    "ping_status",
                    False,
                    f"PING failed ({probe})",
                    _now() - t0,
                )
            st = la.get_status()
            return CheckResult(
                "ping_status",
                True,
                f"state={st.get('state_name')} trig_position={st.get('trig_position')} rle_en={st.get('rle_en')}",
                _now() - t0,
            )
    except Exception as e:
        probe = _uart_probe_once(port, baud, timeout) if deep else "no deep probe"
        return CheckResult(
            "ping_status",
            False,
            f"{type(e).__name__}: {e} ({probe})",
            _now() - t0,
        )


def _check_register_roundtrip(port: str, baud: int, timeout: float) -> CheckResult:
    t0 = _now()
    addr = proto.REG_TRIG_VALUE
    val = 0x00A5_5A3C
    # TRIG_VALUE is PROBE_WIDTH-wide in RTL (16 bits by default),
    # so upper bits are intentionally truncated by hardware.
    exp = val & 0xFFFF
    try:
        with LogicAnalyzerDevice(port, baud=baud, timeout=timeout) as la:
            if not la.ping():
                return CheckResult("register_roundtrip", False, "PING failed", _now() - t0)
            la.set_reg(addr, val)
            got = la.get_reg(addr)
            ok = got == exp
            return CheckResult(
                "register_roundtrip",
                ok,
                f"wrote=0x{val:08X} expected=0x{exp:08X} read=0x{got:08X}",
                _now() - t0,
            )
    except Exception as e:
        return CheckResult("register_roundtrip", False, f"{type(e).__name__}: {e}", _now() - t0)


def _check_capture(port: str, baud: int, timeout: float, rle: bool) -> CheckResult:
    t0 = _now()
    mode = "capture_rle" if rle else "capture_raw"
    try:
        with LogicAnalyzerDevice(port, baud=baud, timeout=timeout) as la:
            if not la.ping():
                return CheckResult(mode, False, "PING failed", _now() - t0)
            cfg = CaptureConfig(
                probe_width=16,
                pre_trig_samples=32,
                post_trig_samples=64,
                trig_mode="IMMEDIATE",
                trig_mask=0,
                trig_value=0,
                rle=rle,
                sample_rate_hz=12_000_000,
            )
            cap = run_capture(la, cfg)
            ok = cap.total_samples > 0 and 0 <= cap.trig_position < cap.total_samples
            if cap.samples:
                detail = (
                    f"samples={cap.total_samples} trig_position={cap.trig_position} "
                    f"first=0x{cap.samples[0]:04X} last=0x{cap.samples[-1]:04X}"
                )
            else:
                detail = f"samples=0 trig_position={cap.trig_position}"
            return CheckResult(mode, ok, detail, _now() - t0)
    except Exception as e:
        return CheckResult(mode, False, f"{type(e).__name__}: {e}", _now() - t0)


def _check_export_vcd_csv(port: str, baud: int, timeout: float) -> CheckResult:
    t0 = _now()
    try:
        with LogicAnalyzerDevice(port, baud=baud, timeout=timeout) as la:
            if not la.ping():
                return CheckResult("export_vcd_csv", False, "PING failed", _now() - t0)

            cfg = CaptureConfig(
                probe_width=16,
                pre_trig_samples=16,
                post_trig_samples=16,
                trig_mode="IMMEDIATE",
                trig_mask=0,
                trig_value=0,
                rle=False,
                sample_rate_hz=12_000_000,
            )
            cap = run_capture(la, cfg)
            if cap.total_samples <= 0:
                return CheckResult("export_vcd_csv", False, "capture returned zero samples", _now() - t0)

            with tempfile.TemporaryDirectory(prefix="la_hw_validate_") as td:
                out_dir = Path(td)
                vcd_path = out_dir / "capture.vcd"
                csv_path = out_dir / "capture.csv"
                export_vcd(cap, vcd_path)
                export_csv(cap, csv_path)

                if (not vcd_path.exists()) or vcd_path.stat().st_size == 0:
                    return CheckResult("export_vcd_csv", False, "VCD file missing/empty", _now() - t0)
                if (not csv_path.exists()) or csv_path.stat().st_size == 0:
                    return CheckResult("export_vcd_csv", False, "CSV file missing/empty", _now() - t0)

                vcd_text = vcd_path.read_text(encoding="utf-8", errors="replace")
                if "$enddefinitions $end" not in vcd_text:
                    return CheckResult("export_vcd_csv", False, "VCD missing enddefinitions", _now() - t0)

                csv_head = csv_path.read_text(encoding="utf-8", errors="replace").splitlines()
                if not csv_head or not csv_head[0].startswith("sample_index,time_ns,raw_hex,is_trigger"):
                    return CheckResult("export_vcd_csv", False, "CSV header malformed", _now() - t0)

                return CheckResult(
                    "export_vcd_csv",
                    True,
                    f"vcd_bytes={vcd_path.stat().st_size} csv_bytes={csv_path.stat().st_size}",
                    _now() - t0,
                )
    except Exception as e:
        return CheckResult("export_vcd_csv", False, f"{type(e).__name__}: {e}", _now() - t0)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Validate FPGA logic analyzer on real hardware.")
    p.add_argument("--port", help="Serial port path (e.g. /dev/ttyUSB1, COM6).")
    p.add_argument("--baud", type=int, default=115_200, help="UART baud rate (default: 115200).")
    p.add_argument("--timeout", type=float, default=1.0, help="Per-operation serial timeout seconds.")
    p.add_argument("--repo-root", default=".", help="Repository root (default: current directory).")
    p.add_argument("--skip-program", action="store_true", help="Skip make synth/prog.")
    p.add_argument("--deep-diagnostics", action="store_true", help="Collect extra UART diagnostics on failure.")
    p.add_argument("--baud-sweep", action="store_true", help="On ping failure, try multiple baud rates and report.")
    p.add_argument("--json-out", help="Optional path to write JSON report.")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    repo = Path(args.repo_root).resolve()

    port = args.port
    if not port:
        detected = _detect_icebreaker_ports()
        if not detected:
            print("FAIL: no iCEBreaker serial ports detected; pass --port explicitly")
            local = _list_local_serial_candidates()
            if local:
                print(f"Detected local serial candidates: {', '.join(local)}")
            else:
                print("No /dev/ttyUSB* or /dev/ttyACM* devices found. If using WSL, re-attach USB device.")
            return 2
        port = detected[-1]
    elif not _port_exists(port):
        print(f"FAIL: requested port does not exist: {port}")
        local = _list_local_serial_candidates()
        if local:
            print(f"Detected local serial candidates: {', '.join(local)}")
        else:
            print("No /dev/ttyUSB* or /dev/ttyACM* devices found. If using WSL, re-attach USB device.")
        return 2

    results: list[CheckResult] = []

    if not args.skip_program:
        results.append(_check_make(repo, "synth"))
        if not results[-1].ok:
            return _finish(results, args.json_out)
        results.append(_check_make(repo, "prog"))
        if not results[-1].ok:
            return _finish(results, args.json_out)

    results.append(_check_ping_status(port, args.baud, args.timeout, args.deep_diagnostics))
    if (not results[-1].ok) and args.baud_sweep:
        t0 = _now()
        sweep = []
        for b in [9600, 19200, 38400, 57600, 74880, 115200, 230400, 460800, 921600, 1_000_000]:
            probe = _uart_probe_once(port, b, min(args.timeout, 0.5))
            sweep.append(f"{b}:{probe}")
        results.append(
            CheckResult(
                "baud_sweep_probe",
                False,
                "; ".join(sweep),
                _now() - t0,
            )
        )
    if results[-1].ok:
        results.append(_check_register_roundtrip(port, args.baud, args.timeout))
        results.append(_check_capture(port, args.baud, args.timeout, rle=False))
        results.append(_check_export_vcd_csv(port, args.baud, args.timeout))
        results.append(_check_capture(port, args.baud, args.timeout, rle=True))

    return _finish(results, args.json_out)


def _finish(results: list[CheckResult], json_out: Optional[str]) -> int:
    print("\nHardware validation report")
    print("=" * 28)
    any_fail = False
    for r in results:
        mark = "PASS" if r.ok else "FAIL"
        if not r.ok:
            any_fail = True
        print(f"[{mark}] {r.name} ({r.elapsed_s:.2f}s)")
        print(f"       {r.detail}")

    payload = {"ok": not any_fail, "results": [asdict(r) for r in results]}
    if json_out:
        out = Path(json_out)
        out.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        print(f"\nWrote JSON report: {out}")

    if any_fail:
        print("\nOverall: FAIL")
        return 1

    print("\nOverall: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
