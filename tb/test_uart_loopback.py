"""
test_uart_loopback.py — cocotb TX→RX loopback test.

Instantiates both uart_tx and uart_rx from the same testbench harness.
Uses a wrapper top-level (tb_loopback.v) that wires tx_pin → rx.
"""

import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

CLK_FREQ      = 12_000_000
BAUD_RATE     = 115_200
CLKS_PER_BIT  = CLK_FREQ // BAUD_RATE
CLK_PERIOD_NS = 1_000_000_000 // CLK_FREQ


async def send_byte(dut, value: int):
    """Send via TX side of loopback."""
    while not dut.tx_ready.value:
        await RisingEdge(dut.clk)
    dut.tx_data.value  = value
    dut.tx_valid.value = 1
    await RisingEdge(dut.clk)
    dut.tx_valid.value = 0


async def recv_byte(dut, timeout_clks: int = CLKS_PER_BIT * 14) -> int:
    """Wait for rx_valid; return rx_data."""
    for _ in range(timeout_clks):
        await RisingEdge(dut.clk)
        if dut.rx_valid.value:
            return int(dut.rx_data.value)
    raise AssertionError("Timeout: rx_valid never asserted")


async def reset_dut(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, units="ns").start())
    dut.tx_data.value  = 0
    dut.tx_valid.value = 0
    dut.rst_n.value    = 0
    await ClockCycles(dut.clk, 4)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 4)


@cocotb.test()
async def test_loopback_all_256(dut):
    """All 256 byte values round-trip TX→RX correctly."""
    await reset_dut(dut)
    for value in range(256):
        await ClockCycles(dut.clk, 5)
        cocotb.start_soon(send_byte(dut, value))
        got = await recv_byte(dut)
        assert got == value, f"Loopback 0x{value:02X}: got 0x{got:02X}"


@cocotb.test()
async def test_loopback_random(dut):
    """32 random values round-trip."""
    await reset_dut(dut)
    rng = random.Random(42)
    for _ in range(32):
        value = rng.randint(0, 255)
        await ClockCycles(dut.clk, 5)
        cocotb.start_soon(send_byte(dut, value))
        got = await recv_byte(dut)
        assert got == value, f"Expected 0x{value:02X} got 0x{got:02X}"
