"""
test_uart_rx.py — cocotb tests for uart_rx module.

Tests: valid bytes, framing error, back-to-back bytes, idle recovery,
       all-256 values, random stimulus.
"""

import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, with_timeout
from cocotb.utils import get_sim_time

CLK_FREQ     = 12_000_000
BAUD_RATE    = 115_200
CLKS_PER_BIT = CLK_FREQ // BAUD_RATE   # 104
CLK_PERIOD_NS = 1_000_000_000 // CLK_FREQ  # ~83 ns


async def uart_send(dut, value: int, bad_stop: bool = False):
    """Drive one 8N1 byte onto dut.rx at BAUD_RATE.

    For a valid stop bit (rx=1), we leave rx as-is after the stop period
    and do NOT write to it again — the line is already idle-high.  This
    avoids a race where a new send starts before this coroutine finishes
    its ClockCycles, and the final write here would corrupt the next start bit.

    For bad_stop (rx=0 during stop), we must restore idle after the period.
    """
    dut.rx.value = 0  # start bit
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    for i in range(8):
        dut.rx.value = (value >> i) & 1
        await ClockCycles(dut.clk, CLKS_PER_BIT)
    dut.rx.value = 0 if bad_stop else 1  # stop bit
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    if bad_stop:
        dut.rx.value = 1  # restore idle only after a bad (low) stop bit


async def wait_valid(dut, timeout_clks: int = CLKS_PER_BIT * 14) -> int:
    """Wait for rx_valid pulse; return rx_data value. Raises on timeout."""
    for _ in range(timeout_clks):
        await RisingEdge(dut.clk)
        if dut.rx_valid.value:
            return int(dut.rx_data.value)
    raise AssertionError("Timeout: rx_valid never asserted")


async def wait_error(dut, timeout_clks: int = CLKS_PER_BIT * 14) -> bool:
    """Wait for rx_error pulse. Returns True if error seen (and not rx_valid)."""
    for _ in range(timeout_clks):
        await RisingEdge(dut.clk)
        if dut.rx_error.value:
            assert not dut.rx_valid.value, "rx_valid must not assert with rx_error"
            return True
        if dut.rx_valid.value:
            raise AssertionError("rx_valid asserted but expected framing error")
    raise AssertionError("Timeout: rx_error never asserted")


async def reset_dut(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, units="ns").start())
    dut.rx.value = 1
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 4)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 4)


# ---------------------------------------------------------------------------
@cocotb.test()
async def test_byte_0x55(dut):
    """Alternating-bit pattern 0x55."""
    await reset_dut(dut)
    cocotb.start_soon(uart_send(dut, 0x55))
    got = await wait_valid(dut)
    assert got == 0x55, f"Expected 0x55 got 0x{got:02X}"


@cocotb.test()
async def test_byte_0xAA(dut):
    """Alternating-bit pattern 0xAA."""
    await reset_dut(dut)
    cocotb.start_soon(uart_send(dut, 0xAA))
    got = await wait_valid(dut)
    assert got == 0xAA, f"Expected 0xAA got 0x{got:02X}"


@cocotb.test()
async def test_byte_0x00(dut):
    """All-zeros."""
    await reset_dut(dut)
    cocotb.start_soon(uart_send(dut, 0x00))
    got = await wait_valid(dut)
    assert got == 0x00, f"Expected 0x00 got 0x{got:02X}"


@cocotb.test()
async def test_byte_0xFF(dut):
    """All-ones."""
    await reset_dut(dut)
    cocotb.start_soon(uart_send(dut, 0xFF))
    got = await wait_valid(dut)
    assert got == 0xFF, f"Expected 0xFF got 0x{got:02X}"


@cocotb.test()
async def test_back_to_back(dut):
    """Two bytes with no idle gap."""
    await reset_dut(dut)
    async def sender():
        await uart_send(dut, 0x12)
        await uart_send(dut, 0x34)
    cocotb.start_soon(sender())
    got0 = await wait_valid(dut)
    assert got0 == 0x12, f"Byte 0: expected 0x12 got 0x{got0:02X}"
    got1 = await wait_valid(dut)
    assert got1 == 0x34, f"Byte 1: expected 0x34 got 0x{got1:02X}"


@cocotb.test()
async def test_framing_error(dut):
    """Bad stop bit must set rx_error and suppress rx_valid."""
    await reset_dut(dut)
    cocotb.start_soon(uart_send(dut, 0x42, bad_stop=True))
    await wait_error(dut)


@cocotb.test()
async def test_recovery_after_framing_error(dut):
    """Valid byte after framing error is received correctly."""
    await reset_dut(dut)
    cocotb.start_soon(uart_send(dut, 0x42, bad_stop=True))
    await wait_error(dut)
    # wait_error returns ~CLKS_PER_BIT/2 into the stop period; the bad_stop
    # coroutine still has ~CLKS_PER_BIT/2 cycles left before it restores rx=1.
    # Wait a full bit period so the previous coroutine finishes cleanly.
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    cocotb.start_soon(uart_send(dut, 0xBE))
    got = await wait_valid(dut)
    assert got == 0xBE, f"Recovery byte: expected 0xBE got 0x{got:02X}"


@cocotb.test()
async def test_all_256_values(dut):
    """Every possible byte value received correctly."""
    await reset_dut(dut)
    for value in range(256):
        await ClockCycles(dut.clk, 5)
        cocotb.start_soon(uart_send(dut, value))
        got = await wait_valid(dut)
        assert got == value, f"Value 0x{value:02X}: got 0x{got:02X}"


@cocotb.test()
async def test_random_bytes(dut):
    """32 random bytes."""
    await reset_dut(dut)
    rng = random.Random(0xDEADBEEF)
    for _ in range(32):
        value = rng.randint(0, 255)
        await ClockCycles(dut.clk, 5)
        cocotb.start_soon(uart_send(dut, value))
        got = await wait_valid(dut)
        assert got == value, f"Expected 0x{value:02X} got 0x{got:02X}"
