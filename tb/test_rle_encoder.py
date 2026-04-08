"""
test_rle_encoder.py — cocotb tests for rle_encoder.

Tests:
  1. Passthrough mode (en=0): samples pass as raw 2-byte MSB-first pairs
  2. All-unique values: every sample emits a 3-byte literal token
  3. All-same values: first literal + run tokens
  4. Long run (>127): requires multiple run tokens
  5. Alternating values: literal each time (no runs)
  6. Mixed run then change
  7. stream_end flushes pending run
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer

CLK_PERIOD_NS = 10  # 100 MHz


async def reset_dut(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, units="ns").start())
    dut.en.value         = 0
    dut.raw_data.value   = 0
    dut.raw_valid.value  = 0
    dut.stream_end.value = 0
    dut.enc_ready.value  = 1   # consumer always ready unless test overrides
    dut.rst_n.value      = 0
    await ClockCycles(dut.clk, 4)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 4)


async def send_sample(dut, value: int):
    """Present one raw sample and wait for raw_ready."""
    dut.raw_data.value  = value
    dut.raw_valid.value = 1
    while True:
        await RisingEdge(dut.clk)
        if dut.raw_ready.value:
            break
    dut.raw_valid.value = 0
    dut.raw_data.value  = 0


async def collect_bytes(dut, n_bytes: int, timeout_cycles: int = 5000) -> list:
    """Collect n_bytes from enc_byte/enc_valid, consumer always ready."""
    result = []
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if dut.enc_valid.value:
            result.append(int(dut.enc_byte.value))
            if len(result) == n_bytes:
                return result
    raise AssertionError(
        f"Timeout: only {len(result)}/{n_bytes} bytes collected, got={result}"
    )


async def send_and_collect(dut, samples: list, extra_wait: int = 20) -> list:
    """Send all samples then stream_end; collect all output bytes.

    The idle timeout only starts counting AFTER the producer finishes,
    so long runs with no mid-stream output don't cause early exit.
    """
    collected = []

    async def producer():
        for s in samples:
            await send_sample(dut, s)
        await RisingEdge(dut.clk)
        dut.stream_end.value = 1
        await RisingEdge(dut.clk)
        dut.stream_end.value = 0

    prod = cocotb.start_soon(producer())

    idle = 0
    while True:
        await RisingEdge(dut.clk)
        if dut.enc_valid.value:
            collected.append(int(dut.enc_byte.value))
            idle = 0
        else:
            if prod.done():
                idle += 1
        if prod.done() and idle >= extra_wait:
            break

    return collected


# ---------------------------------------------------------------------------
@cocotb.test()
async def test_passthrough_single(dut):
    """Passthrough (en=0): one 0x1234 sample → [0x12, 0x34]."""
    await reset_dut(dut)
    dut.en.value = 0

    cocotb.start_soon(send_sample(dut, 0x1234))
    raw = await collect_bytes(dut, 2)
    assert raw == [0x12, 0x34], f"passthrough: got {[hex(b) for b in raw]}"


@cocotb.test()
async def test_passthrough_multiple(dut):
    """Passthrough: 4 distinct samples → 8 bytes MSB-first."""
    await reset_dut(dut)
    dut.en.value = 0
    values = [0xABCD, 0x1234, 0x0000, 0xFFFF]

    async def producer():
        for v in values:
            await send_sample(dut, v)

    cocotb.start_soon(producer())
    raw = await collect_bytes(dut, 8)
    expected = []
    for v in values:
        expected += [(v >> 8) & 0xFF, v & 0xFF]
    assert raw == expected, f"passthrough multi: {[hex(b) for b in raw]}"


@cocotb.test()
async def test_rle_all_unique(dut):
    """RLE: 4 unique values → 4 literal tokens (12 bytes, no runs)."""
    await reset_dut(dut)
    dut.en.value = 1
    values = [0x0001, 0x0002, 0x0003, 0x0004]

    raw = await send_and_collect(dut, values)
    # Each literal: [0x00, hi, lo]
    assert len(raw) == 12, f"unique: expected 12 bytes, got {len(raw)}: {[hex(b) for b in raw]}"
    for i, v in enumerate(values):
        assert raw[i*3]   == 0x00,        f"literal[{i}] marker != 0x00"
        assert raw[i*3+1] == (v >> 8) & 0xFF, f"literal[{i}] hi mismatch"
        assert raw[i*3+2] == v & 0xFF,    f"literal[{i}] lo mismatch"


@cocotb.test()
async def test_rle_all_same(dut):
    """RLE: 5 identical values → 1 literal + 1 run token (count=4)."""
    await reset_dut(dut)
    dut.en.value = 1
    value = 0xBEEF
    values = [value] * 5

    raw = await send_and_collect(dut, values)
    # Expected: [0x00, 0xBE, 0xEF, 0x84]  (0x80|4 = 0x84)
    assert len(raw) == 4, f"all-same: expected 4 bytes, got {len(raw)}: {[hex(b) for b in raw]}"
    assert raw[0] == 0x00,            "literal marker"
    assert raw[1] == 0xBE,            "literal hi"
    assert raw[2] == 0xEF,            "literal lo"
    assert raw[3] == (0x80 | 4),      f"run token: got {hex(raw[3])}"


@cocotb.test()
async def test_rle_long_run(dut):
    """RLE: 130 identical values → 1 literal + 2 run tokens (127 + 2)."""
    await reset_dut(dut)
    dut.en.value = 1
    value = 0x1234
    values = [value] * 130

    raw = await send_and_collect(dut, values, extra_wait=100)
    # 1 literal (3 bytes) + run of 127 (1 byte) + run of 2 (1 byte) = 5 bytes
    assert len(raw) == 5, f"long-run: expected 5 bytes, got {len(raw)}: {[hex(b) for b in raw]}"
    assert raw[0] == 0x00,            f"literal marker: {hex(raw[0])}"
    assert raw[1] == 0x12,            f"hi byte: {hex(raw[1])}"
    assert raw[2] == 0x34,            f"lo byte: {hex(raw[2])}"
    assert raw[3] == (0x80 | 127),    f"run1: {hex(raw[3])}"
    assert raw[4] == (0x80 | 2),      f"run2: {hex(raw[4])}"


@cocotb.test()
async def test_rle_alternating(dut):
    """RLE: alternating A/B values → all literals (6 bytes for 2 values)."""
    await reset_dut(dut)
    dut.en.value = 1
    values = [0xAAAA, 0x5555, 0xAAAA, 0x5555]

    raw = await send_and_collect(dut, values)
    assert len(raw) == 12, f"alternating: expected 12 bytes, got {len(raw)}"
    for i, v in enumerate(values):
        assert raw[i*3]   == 0x00,            f"lit[{i}] marker"
        assert raw[i*3+1] == (v >> 8) & 0xFF, f"lit[{i}] hi"
        assert raw[i*3+2] == v & 0xFF,        f"lit[{i}] lo"


@cocotb.test()
async def test_rle_run_then_change(dut):
    """RLE: 3× val_A then val_B → literal A, run(2), literal B."""
    await reset_dut(dut)
    dut.en.value = 1
    A, B = 0x1111, 0x2222
    values = [A, A, A, B]

    raw = await send_and_collect(dut, values)
    # literal A (3) + run(2) (1) + literal B (3) = 7 bytes
    assert len(raw) == 7, f"run-change: expected 7 bytes, got {len(raw)}: {[hex(b) for b in raw]}"
    assert raw[0:3]  == [0x00, 0x11, 0x11], f"lit A: {raw[0:3]}"
    assert raw[3]    == (0x80 | 2),          f"run: {hex(raw[3])}"
    assert raw[4:7]  == [0x00, 0x22, 0x22], f"lit B: {raw[4:7]}"


@cocotb.test()
async def test_rle_stream_end_flushes(dut):
    """stream_end on a pending run flushes the run token."""
    await reset_dut(dut)
    dut.en.value = 1
    value = 0xCAFE
    values = [value, value, value]  # literal + run(2)

    raw = await send_and_collect(dut, values)
    # Without stream_end, the encoder might hold the run token.
    # With stream_end, it must emit it.
    assert len(raw) == 4, f"flush: expected 4 bytes, got {len(raw)}: {[hex(b) for b in raw]}"
    assert raw[3] == (0x80 | 2), f"flushed run: {hex(raw[3])}"


@cocotb.test()
async def test_passthrough_backpressure(dut):
    """Passthrough with enc_ready toggling: all bytes must still arrive."""
    await reset_dut(dut)
    dut.en.value = 1
    dut.enc_ready.value = 0  # start with backpressure

    value = 0xDEAD
    collected = []

    async def slow_consumer():
        for _ in range(500):
            await RisingEdge(dut.clk)
            # toggle ready every 3 cycles
            dut.enc_ready.value = (1 if (_ % 3 == 0) else 0)
            if dut.enc_valid.value and dut.enc_ready.value:
                collected.append(int(dut.enc_byte.value))
            if len(collected) >= 3:
                break
        dut.enc_ready.value = 1

    cocotb.start_soon(slow_consumer())
    await send_sample(dut, value)
    # pulse stream_end
    await RisingEdge(dut.clk)
    dut.stream_end.value = 1
    await RisingEdge(dut.clk)
    dut.stream_end.value = 0
    await ClockCycles(dut.clk, 50)

    assert len(collected) >= 3, f"backpressure: only got {len(collected)} bytes"
    # For a single unique value in RLE: 3 bytes literal
    assert collected[0] == 0x00
    assert collected[1] == 0xDE
    assert collected[2] == 0xAD
