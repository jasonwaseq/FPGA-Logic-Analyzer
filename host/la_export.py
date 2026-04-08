"""
la_export.py — Export captured data to VCD and CSV formats.

VCD (Value Change Dump) is compatible with GTKWave and other waveform viewers.
CSV is suitable for spreadsheet analysis.

Usage:
    from host.la_export import export_vcd, export_csv
    export_vcd(capture, "capture.vcd", probe_names=["CLK","DATA","CS","IRQ"])
    export_csv(capture, "capture.csv")
"""

from __future__ import annotations

import csv
import datetime
from pathlib import Path
from typing import Optional, Sequence

from .la_capture import Capture


# ---------------------------------------------------------------------------
# VCD export
# ---------------------------------------------------------------------------
def export_vcd(
    capture: Capture,
    filename: str | Path,
    probe_names: Optional[list[str]] = None,
    timescale: str = "1ns",
) -> None:
    """
    Write a GTKWave-compatible VCD file.

    Each probe bit gets its own signal variable.  A virtual 1-bit
    'trigger_marker' signal pulses high at the trigger sample.

    Parameters
    ----------
    capture : Capture
        Completed capture data.
    filename : str or Path
        Output file path.
    probe_names : list[str], optional
        Names for each probe bit (probe_names[0] = bit 0).
        Defaults to "probe[N]" if not provided.
    timescale : str
        VCD timescale string, e.g. "1ns", "10ns".
    """
    w = capture.config.probe_width
    names = probe_names or capture.config.probe_names or [f"probe[{i}]" for i in range(w)]
    # Pad or truncate to exactly w names
    while len(names) < w:
        names.append(f"probe[{len(names)}]")
    names = names[:w]

    sample_period_ns = 1e9 / capture.config.sample_rate_hz

    # VCD identifier characters (printable ASCII 33–126, excluding space)
    def _id(n: int) -> str:
        chars = [chr(c) for c in range(33, 127)]
        result = ""
        while True:
            result = chars[n % len(chars)] + result
            n //= len(chars)
            if n == 0:
                break
        return result

    with open(filename, "w", newline="\n") as f:
        # Header
        f.write(f"$date {datetime.datetime.now().isoformat()} $end\n")
        f.write(f"$version FPGA Logic Analyzer v1.0 $end\n")
        f.write(f"$timescale {timescale} $end\n")
        f.write("$scope module logic_analyzer $end\n")

        # Declare all probe bits + trigger marker
        for i in range(w):
            f.write(f"$var wire 1 {_id(i)} {names[i]} $end\n")
        trigger_id = _id(w)
        f.write(f"$var wire 1 {trigger_id} trigger_marker $end\n")

        f.write("$upscope $end\n")
        f.write("$enddefinitions $end\n")

        # Comment indicating trigger position
        trig_time_ns = int(capture.trig_position * sample_period_ns)
        f.write(f"$comment trigger_at_sample {capture.trig_position} "
                f"(t={trig_time_ns}ns) $end\n")

        # Initial values at t=0
        f.write("#0\n")
        f.write("$dumpvars\n")
        first = capture.samples[0] if capture.samples else 0
        for i in range(w):
            f.write(f"b{(first >> i) & 1} {_id(i)}\n")
        f.write(f"b0 {trigger_id}\n")
        f.write("$end\n")

        # Change events
        prev = capture.samples[0] if capture.samples else 0
        for idx, sample in enumerate(capture.samples):
            if idx == 0:
                continue
            t_ns = int(idx * sample_period_ns)
            changed = sample ^ prev
            trig_pulse = (idx == capture.trig_position)

            if changed or trig_pulse:
                f.write(f"#{t_ns}\n")
                for i in range(w):
                    if (changed >> i) & 1:
                        f.write(f"b{(sample >> i) & 1} {_id(i)}\n")
                if trig_pulse:
                    f.write(f"b1 {trigger_id}\n")

            if idx == capture.trig_position:
                # Deassert trigger marker one sample later
                next_t = int((idx + 1) * sample_period_ns)
                f.write(f"#{next_t}\n")
                f.write(f"b0 {trigger_id}\n")

            prev = sample

    print(f"VCD written to {filename}  ({len(capture.samples)} samples, "
          f"trigger at sample {capture.trig_position})")


# ---------------------------------------------------------------------------
# CSV export
# ---------------------------------------------------------------------------
def export_csv(
    capture: Capture,
    filename: str | Path,
    probe_names: Optional[list[str]] = None,
) -> None:
    """
    Write a CSV file with one row per sample.

    Columns: sample_index, time_ns, raw_hex, then one column per probe bit.

    Parameters
    ----------
    capture : Capture
        Completed capture data.
    filename : str or Path
        Output file path.
    probe_names : list[str], optional
        Names for each probe bit.
    """
    w = capture.config.probe_width
    names = probe_names or capture.config.probe_names or [f"probe[{i}]" for i in range(w)]
    while len(names) < w:
        names.append(f"probe[{len(names)}]")
    names = names[:w]

    sample_period_ns = 1e9 / capture.config.sample_rate_hz

    with open(filename, "w", newline="") as f:
        writer = csv.writer(f)

        # Header row
        header = ["sample_index", "time_ns", "raw_hex", "is_trigger"] + names
        writer.writerow(header)

        # Data rows
        for idx, sample in enumerate(capture.samples):
            t_ns = (idx - capture.trig_position) * sample_period_ns
            is_trig = (idx == capture.trig_position)
            bits = [(sample >> i) & 1 for i in range(w)]
            writer.writerow(
                [idx, f"{t_ns:.1f}", f"0x{sample:04X}", int(is_trig)] + bits
            )

    print(f"CSV written to {filename}  ({len(capture.samples)} rows)")
