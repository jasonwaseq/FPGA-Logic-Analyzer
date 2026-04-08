"""
la_capture.py — Capture orchestration and data model.

CaptureConfig  — configuration parameters for one acquisition
Capture        — result of a completed acquisition (samples + metadata)
run_capture()  — high-level: configure device, arm, wait, fetch, return Capture
"""

from __future__ import annotations

import datetime
from dataclasses import dataclass, field
from typing import Optional

from . import la_protocol as proto
from .la_device import LogicAnalyzerDevice


# ---------------------------------------------------------------------------
# Configuration dataclass
# ---------------------------------------------------------------------------
@dataclass
class CaptureConfig:
    """Parameters for a single logic analyzer acquisition."""

    probe_width: int = 16
    """Width of the probe bus in bits."""

    buffer_depth: int = 1024
    """Total capture buffer depth (must match firmware parameter)."""

    pre_trig_samples: int = 256
    """Number of samples to keep before the trigger."""

    post_trig_samples: int = 767
    """Number of samples to capture after the trigger (triggers sample counts as 1)."""

    trig_mode: str = "IMMEDIATE"
    """Trigger mode: 'IMMEDIATE', 'EQUALITY', 'RISING', 'FALLING'."""

    trig_mask: int = 0xFFFF
    """Bitmask selecting which probe bits participate in trigger logic."""

    trig_value: int = 0x0000
    """Reference value for EQUALITY trigger mode."""

    rle: bool = False
    """Use RLE compression for readout transfer."""

    sample_rate_hz: int = 12_000_000
    """Sample clock frequency in Hz (for time annotation)."""

    probe_names: Optional[list[str]] = None
    """Optional list of signal names for display/export."""

    @property
    def total_samples(self) -> int:
        return self.pre_trig_samples + 1 + self.post_trig_samples

    def probe_name(self, bit: int) -> str:
        if self.probe_names and bit < len(self.probe_names):
            return self.probe_names[bit]
        return f"probe[{bit}]"


# ---------------------------------------------------------------------------
# Result dataclass
# ---------------------------------------------------------------------------
@dataclass
class Capture:
    """Result of a completed logic analyzer acquisition."""

    samples: list[int]
    """List of raw sample values (integers), time-ordered."""

    trig_position: int
    """Index of the trigger sample in *samples* (samples[trig_position] is T=0)."""

    config: CaptureConfig = field(default_factory=CaptureConfig)
    """Configuration used for this capture."""

    timestamp: datetime.datetime = field(default_factory=datetime.datetime.now)
    """Wall-clock time when the capture was fetched."""

    @property
    def total_samples(self) -> int:
        return len(self.samples)

    def time_ns(self, index: int) -> float:
        """Convert sample index to time in nanoseconds relative to trigger."""
        dt_ns = 1e9 / self.config.sample_rate_hz
        return (index - self.trig_position) * dt_ns

    def bit_value(self, index: int, bit: int) -> int:
        """Extract one probe bit from a sample."""
        return (self.samples[index] >> bit) & 1

    def channel_values(self, bit: int) -> list[int]:
        """Extract all values for one probe bit as a list."""
        return [(s >> bit) & 1 for s in self.samples]


# ---------------------------------------------------------------------------
# run_capture()
# ---------------------------------------------------------------------------
def run_capture(device: LogicAnalyzerDevice, config: CaptureConfig) -> Capture:
    """
    Perform a complete acquisition:
      1. Write all configuration registers
      2. Set RLE mode
      3. ARM
      4. Poll STATUS until COMPLETE
      5. READ_DATA and decode
      6. Return a Capture object

    Parameters
    ----------
    device : LogicAnalyzerDevice
        An open LogicAnalyzerDevice instance.
    config : CaptureConfig
        Acquisition parameters.

    Returns
    -------
    Capture
        The captured data with metadata.
    """
    # Step 0: force a clean IDLE baseline between captures.
    # This avoids cross-capture state leakage (notably in RLE readout paths).
    device.reset()

    # Step 1: configure registers
    trig_mode_code = proto.TRIG_FROM_NAME.get(config.trig_mode.upper())
    if trig_mode_code is None:
        raise ValueError(
            f"Unknown trigger mode: '{config.trig_mode}'. "
            f"Valid modes: {list(proto.TRIG_FROM_NAME)}"
        )

    device.set_reg(proto.REG_TRIG_MODE,       trig_mode_code)
    device.set_reg(proto.REG_TRIG_MASK,       config.trig_mask  & 0xFFFF)
    device.set_reg(proto.REG_TRIG_VALUE,      config.trig_value & 0xFFFF)
    device.set_reg(proto.REG_PRE_TRIG_DEPTH,  config.pre_trig_samples)
    device.set_reg(proto.REG_POST_TRIG_DEPTH, config.post_trig_samples)
    device.set_reg(proto.REG_RLE_EN,          int(config.rle))

    # Step 2: ARM
    device.arm()

    # Step 3: Wait for completion
    status = device.wait_for_complete(timeout=60.0)
    trig_position = status["trig_position"]

    # Step 4: Fetch and decode data
    samples = device.read_data_decoded(rle=config.rle)

    return Capture(
        samples=samples,
        trig_position=trig_position,
        config=config,
        timestamp=datetime.datetime.now(),
    )
