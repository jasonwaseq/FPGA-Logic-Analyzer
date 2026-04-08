"""
la_display.py — Terminal ASCII waveform renderer.

Renders each probe bit as one row of characters, decimated to fit the
terminal width.  The trigger position is marked with a '|T|' annotation.

Usage:
    from host.la_display import display_waveforms
    display_waveforms(capture, channels=[0,1,2,3], probe_names=["CLK","MOSI","CS","IRQ"])
"""

from __future__ import annotations

import os
import shutil
from typing import Optional

from .la_capture import Capture


# ---------------------------------------------------------------------------
# Rendering characters
# ---------------------------------------------------------------------------
CHAR_HIGH    = "▀"   # upper half-block: signal high
CHAR_LOW     = "▄"   # lower half-block: signal low
CHAR_RISE    = "╱"   # rising edge
CHAR_FALL    = "╲"   # falling edge
CHAR_TRIGGER = "T"   # trigger marker column

# Fallback for terminals without Unicode
CHAR_HIGH_ASCII    = "-"
CHAR_LOW_ASCII     = "_"
CHAR_RISE_ASCII    = "/"
CHAR_FALL_ASCII    = "\\"
CHAR_TRIGGER_ASCII = "T"


def display_waveforms(
    capture: Capture,
    channels: Optional[list[int]] = None,
    width: Optional[int] = None,
    probe_names: Optional[list[str]] = None,
    ascii_only: bool = False,
    show_hex: bool = True,
) -> None:
    """
    Print ASCII waveforms to stdout.

    Parameters
    ----------
    capture : Capture
        Completed capture object.
    channels : list[int], optional
        Which probe bit indices to display.  Default: all bits.
    width : int, optional
        Terminal width in columns.  Default: auto-detect.
    probe_names : list[str], optional
        Signal names per bit.
    ascii_only : bool
        Use plain ASCII characters instead of Unicode block elements.
    show_hex : bool
        Print hex value of each sample at the bottom.
    """
    w = capture.config.probe_width
    if channels is None:
        channels = list(range(w))
    if width is None:
        width = shutil.get_terminal_size((80, 24)).columns

    names = probe_names or capture.config.probe_names or [f"probe[{i}]" for i in range(w)]
    while len(names) < w:
        names.append(f"probe[{len(names)}]")

    # Label width: longest name + 2
    label_w = max(len(names[c]) for c in channels) + 2
    wave_w  = width - label_w - 1   # columns available for waveform

    if wave_w < 4:
        print("[display: terminal too narrow]")
        return

    total = len(capture.samples)
    trig  = capture.trig_position

    # Decimation: map wave_w columns → total samples
    def sample_at_col(col: int) -> int:
        idx = int(col * total / wave_w)
        return min(idx, total - 1)

    def trig_col() -> int:
        return int(trig * wave_w / total)

    # Choose characters
    if ascii_only:
        H, L, R, F, T = "−", "_", "/", "\\", "T"
    else:
        H, L, R, F, T = CHAR_HIGH, CHAR_LOW, CHAR_RISE, CHAR_FALL, CHAR_TRIGGER

    # -----------------------------------------------------------------------
    # Header: time axis
    # -----------------------------------------------------------------------
    sample_period_ns = 1e9 / capture.config.sample_rate_hz
    pre_ns  = int(trig * sample_period_ns)
    post_ns = int((total - trig) * sample_period_ns)
    tc = trig_col()

    print()
    print(f"  Logic Analyzer Capture  "
          f"[{total} samples @ {capture.config.sample_rate_hz/1e6:.0f} MHz  "
          f"| pre={pre_ns/1000:.1f}µs  post={post_ns/1000:.1f}µs  "
          f"| {capture.timestamp.strftime('%H:%M:%S')}]")
    print()

    # -----------------------------------------------------------------------
    # Waveform rows
    # -----------------------------------------------------------------------
    for bit in channels:
        label = names[bit].rjust(label_w)
        row   = []
        prev_val = None

        for col in range(wave_w):
            idx = sample_at_col(col)
            val = (capture.samples[idx] >> bit) & 1

            if col == tc:
                # Trigger column
                row.append(T)
            elif prev_val is None or val == prev_val:
                row.append(H if val else L)
            elif val == 1:
                row.append(R)  # 0→1 rising
            else:
                row.append(F)  # 1→0 falling

            prev_val = val

        print(f"{label} │{''.join(row)}│")

    # -----------------------------------------------------------------------
    # Time ruler
    # -----------------------------------------------------------------------
    ruler = [" "] * wave_w
    if 0 <= tc < wave_w:
        ruler[tc] = "^"
    print(" " * label_w + " │" + "".join(ruler) + "│")
    print(" " * (label_w + 1 + tc) + " T=0")

    # -----------------------------------------------------------------------
    # Optional hex dump (first 16 and last 4 samples)
    # -----------------------------------------------------------------------
    if show_hex and total > 0:
        print()
        n_show = min(16, total)
        hex_str = " ".join(f"{s:04X}" for s in capture.samples[:n_show])
        if total > n_show:
            hex_str += f" ... [{total - n_show} more]"
        print(f"  Samples[0..{n_show-1}]: {hex_str}")
        trig_val = capture.samples[trig] if trig < total else None
        if trig_val is not None:
            print(f"  Trigger sample [{trig}]: 0x{trig_val:04X}")

    print()


# ---------------------------------------------------------------------------
# Simple summary print (no waveform)
# ---------------------------------------------------------------------------
def print_status(status: dict) -> None:
    """Pretty-print a CMD_STATUS response dict."""
    print(f"  State:          {status.get('state_name', '?')}")
    print(f"  Trigger at:     sample {status.get('trig_position', '?')}")
    print(f"  RLE mode:       {'enabled' if status.get('rle_en') else 'disabled'}")
