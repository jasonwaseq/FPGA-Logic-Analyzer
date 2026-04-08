#!/usr/bin/env python3
"""
la_cli.py — Command-line interface for the FPGA Logic Analyzer.

Usage examples:
    # Capture with immediate trigger, dump to VCD
    python -m host.la_cli capture --port /dev/ttyUSB0 --out capture.vcd

    # Rising edge trigger on bit 0, show waveform in terminal
    python -m host.la_cli capture --port COM3 \\
        --trig-mode rising --trig-mask 0x0001 \\
        --pre 256 --post 512 --display

    # Query device status
    python -m host.la_cli status --port /dev/ttyUSB0

    # Reset (abort) device
    python -m host.la_cli reset --port /dev/ttyUSB0

    # Show a previously saved VCD file
    python -m host.la_cli show capture.vcd
"""

from __future__ import annotations

import argparse
import sys
import os
from pathlib import Path

# Allow running as 'python -m host.la_cli' or 'python host/la_cli.py'
if __name__ == "__main__" and __package__ is None:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    __package__ = "host"

from . import la_protocol as proto
from .la_device import LogicAnalyzerDevice
from .la_capture import Capture, CaptureConfig, run_capture
from .la_export import export_vcd, export_csv
from .la_display import display_waveforms, print_status


# ---------------------------------------------------------------------------
# Shared argument parser helpers
# ---------------------------------------------------------------------------
def add_port_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--port", "-p",
        required=True,
        help="Serial port (e.g. /dev/ttyUSB0 or COM3)",
    )
    parser.add_argument(
        "--baud", "-b",
        type=int,
        default=115_200,
        help="Baud rate (default: 115200)",
    )


def add_probe_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--probe-names",
        metavar="NAME",
        nargs="*",
        help="Signal names for probe bits, LSB first (e.g. CLK MOSI MISO CS)",
    )
    parser.add_argument(
        "--probe-width",
        type=int,
        default=16,
        help="Probe bus width in bits (default: 16)",
    )


# ---------------------------------------------------------------------------
# 'capture' subcommand
# ---------------------------------------------------------------------------
def cmd_capture(args: argparse.Namespace) -> int:
    config = CaptureConfig(
        probe_width      = args.probe_width,
        pre_trig_samples = args.pre,
        post_trig_samples= args.post,
        trig_mode        = args.trig_mode.upper(),
        trig_mask        = int(args.trig_mask, 0),
        trig_value       = int(args.trig_value, 0),
        rle              = args.rle,
        sample_rate_hz   = args.sample_rate,
        probe_names      = args.probe_names,
    )

    print(f"Connecting to {args.port} @ {args.baud} baud...")
    with LogicAnalyzerDevice(args.port, baud=args.baud, probe_width=args.probe_width) as la:
        if not la.ping():
            print("ERROR: No response from device (PING failed)", file=sys.stderr)
            return 1
        print("Device OK")

        print(f"Configuring: trig={config.trig_mode}  "
              f"mask=0x{config.trig_mask:04X}  value=0x{config.trig_value:04X}  "
              f"pre={config.pre_trig_samples}  post={config.post_trig_samples}  "
              f"rle={'on' if config.rle else 'off'}")

        print("Arming...")
        try:
            capture = run_capture(la, config)
        except TimeoutError as e:
            print(f"ERROR: {e}", file=sys.stderr)
            return 1

        print(f"Capture complete: {capture.total_samples} samples  "
              f"trigger at [{capture.trig_position}]")

        # Export
        if args.out:
            out_path = Path(args.out)
            suffix = out_path.suffix.lower()
            if suffix == ".vcd":
                export_vcd(capture, out_path, probe_names=args.probe_names)
            elif suffix == ".csv":
                export_csv(capture, out_path, probe_names=args.probe_names)
            else:
                # Default: VCD
                export_vcd(capture, out_path, probe_names=args.probe_names)

        if args.csv:
            export_csv(capture, args.csv, probe_names=args.probe_names)

        if args.vcd and args.vcd != args.out:
            export_vcd(capture, args.vcd, probe_names=args.probe_names)

        # Terminal display
        if args.display:
            channels = None
            if args.channels:
                channels = [int(c) for c in args.channels.split(",")]
            display_waveforms(
                capture,
                channels=channels,
                probe_names=args.probe_names,
                ascii_only=args.ascii,
            )

    return 0


# ---------------------------------------------------------------------------
# 'status' subcommand
# ---------------------------------------------------------------------------
def cmd_status(args: argparse.Namespace) -> int:
    with LogicAnalyzerDevice(args.port, baud=args.baud) as la:
        if not la.ping():
            print("ERROR: No response from device", file=sys.stderr)
            return 1
        status = la.get_status()
        print("Device status:")
        print_status(status)
    return 0


# ---------------------------------------------------------------------------
# 'reset' subcommand
# ---------------------------------------------------------------------------
def cmd_reset(args: argparse.Namespace) -> int:
    with LogicAnalyzerDevice(args.port, baud=args.baud) as la:
        la.reset()
        print("Reset OK")
    return 0


# ---------------------------------------------------------------------------
# 'show' subcommand (offline waveform display from file)
# ---------------------------------------------------------------------------
def cmd_show(args: argparse.Namespace) -> int:
    """
    Minimal offline viewer: re-reads a CSV export and displays it.
    (Full VCD parsing is out of scope; use GTKWave for VCD files.)
    """
    import csv as csv_mod
    path = Path(args.file)
    if not path.exists():
        print(f"ERROR: file not found: {path}", file=sys.stderr)
        return 1
    if path.suffix.lower() != ".csv":
        print(f"Tip: Open {path} in GTKWave for VCD waveform viewing.")
        print("(This viewer only supports CSV files.)")
        return 0

    # Read CSV
    samples = []
    trig_pos = 0
    with open(path, newline="") as f:
        reader = csv_mod.DictReader(f)
        for row in reader:
            samples.append(int(row["raw_hex"], 16))
            if int(row.get("is_trigger", 0)):
                trig_pos = int(row["sample_index"])

    if not samples:
        print("ERROR: No samples in CSV", file=sys.stderr)
        return 1

    # Build a minimal Capture for display
    from .la_capture import Capture, CaptureConfig
    cfg = CaptureConfig(
        probe_width  = args.probe_width,
        probe_names  = args.probe_names,
        sample_rate_hz = args.sample_rate,
    )
    cap = Capture(samples=samples, trig_position=trig_pos, config=cfg)

    channels = None
    if args.channels:
        channels = [int(c) for c in args.channels.split(",")]
    display_waveforms(cap, channels=channels, probe_names=args.probe_names,
                      ascii_only=args.ascii)
    return 0


# ---------------------------------------------------------------------------
# Argument parser construction
# ---------------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="la",
        description="FPGA Logic Analyzer host tool",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # ---- capture ----
    p_cap = sub.add_parser("capture", help="Run a capture and save/display results")
    add_port_args(p_cap)
    add_probe_args(p_cap)
    p_cap.add_argument("--pre",  type=int, default=256, help="Pre-trigger samples (default 256)")
    p_cap.add_argument("--post", type=int, default=767, help="Post-trigger samples (default 767)")
    p_cap.add_argument("--trig-mode",  default="IMMEDIATE",
                       choices=["IMMEDIATE","EQUALITY","RISING","FALLING"],
                       help="Trigger mode (default IMMEDIATE)")
    p_cap.add_argument("--trig-mask",  default="0xFFFF", help="Trigger mask hex (default 0xFFFF)")
    p_cap.add_argument("--trig-value", default="0x0000", help="Trigger value hex (default 0x0000)")
    p_cap.add_argument("--rle", action="store_true", help="Enable RLE compression for transfer")
    p_cap.add_argument("--sample-rate", type=int, default=12_000_000,
                       help="Sample rate Hz for time annotation (default 12000000)")
    p_cap.add_argument("--out",   metavar="FILE", help="Output file (.vcd or .csv)")
    p_cap.add_argument("--vcd",   metavar="FILE", help="Also save VCD file")
    p_cap.add_argument("--csv",   metavar="FILE", help="Also save CSV file")
    p_cap.add_argument("--display", action="store_true", help="Show ASCII waveform in terminal")
    p_cap.add_argument("--channels", metavar="0,1,2", help="Channels to display (comma-separated bit indices)")
    p_cap.add_argument("--ascii", action="store_true", help="Use ASCII-only waveform characters")

    # ---- status ----
    p_sts = sub.add_parser("status", help="Query device status")
    add_port_args(p_sts)

    # ---- reset ----
    p_rst = sub.add_parser("reset", help="Abort any capture and reset device to IDLE")
    add_port_args(p_rst)

    # ---- show ----
    p_shw = sub.add_parser("show", help="Display a saved CSV capture file")
    p_shw.add_argument("file", help="Path to CSV file")
    add_probe_args(p_shw)
    p_shw.add_argument("--sample-rate", type=int, default=12_000_000)
    p_shw.add_argument("--channels", metavar="0,1,2")
    p_shw.add_argument("--ascii", action="store_true")

    return parser


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    dispatch = {
        "capture": cmd_capture,
        "status":  cmd_status,
        "reset":   cmd_reset,
        "show":    cmd_show,
    }
    fn = dispatch.get(args.command)
    if fn is None:
        parser.print_help()
        return 1
    return fn(args)


if __name__ == "__main__":
    sys.exit(main())
