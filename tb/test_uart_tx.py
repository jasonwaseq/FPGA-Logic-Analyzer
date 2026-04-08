"""
test_uart_tx.py — cocotb tests for uart_tx module.

Tests: single byte, back-to-back, tx_ready handshake, all-256 values.
"""

import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles

CLK_FREQ      = 12_000_000
BAUD_RATE     = 115_200
CLKS_PER_BIT  = CLK_FREQ // BAUD_RATE
CLK_PERIOD_NS = 1_000_000_000 // CLK_FREQ


async def tx_send(dut, value: int):
    """Write one byte via DUT tx interface; waits for tx_ready."""
    # Wait for tx_ready
    while not dut.tx_ready.value:
        await RisingEdge(dut.clk)
    dut.tx_data.value = value
    dut.tx_valid.value = 1
    await RisingEdge(dut.clk)
    dut.tx_valid.value = 0


async def tx_decode(dut) -> int:
    """Sample the tx_pin bitstream and decode one 8N1 byte."""
    # Wait for start bit (falling edge)
    while dut.tx_pin.value != 0:
        await RisingEdge(dut.clk)
    # Skip to mid-point of start bit
    await ClockCycles(dut.clk, CLKS_PER_BIT // 2)
    assert dut.tx_pin.value == 0, "Start bit must be low at mid-sample"
    # Sample 8 data bits
    bits = 0
    for i in range(8):
        await ClockCycles(dut.clk, CLKS_PER_BIT)
        bits |= (int(dut.tx_pin.value) << i)
    # Sample stop bit
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    assert dut.tx_pin.value == 1, "Stop bit must be high"
    return bits


async def reset_dut(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, units="ns").start())
    dut.tx_data.value  = 0
    dut.tx_valid.value = 0
    dut.rst_n.value    = 0
    await ClockCycles(dut.clk, 4)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 4)


# ---------------------------------------------------------------------------
@cocotb.test()
async def test_single_byte_0x55(dut):
    await reset_dut(dut)
    cocotb.start_soon(tx_send(dut, 0x55))
    got = await tx_decode(dut)
    assert got == 0x55, f"Expected 0x55 got 0x{got:02X}"


@cocotb.test()
async def test_single_byte_0xAA(dut):
    await reset_dut(dut)
    cocotb.start_soon(tx_send(dut, 0xAA))
    got = await tx_decode(dut)
    assert got == 0xAA, f"Expected 0xAA got 0x{got:02X}"


@cocotb.test()
async def test_all_zeros(dut):
    await reset_dut(dut)
    cocotb.start_soon(tx_send(dut, 0x00))
    got = await tx_decode(dut)
    assert got == 0x00, f"Expected 0x00 got 0x{got:02X}"


@cocotb.test()
async def test_all_ones(dut):
    await reset_dut(dut)
    cocotb.start_soon(tx_send(dut, 0xFF))
    got = await tx_decode(dut)
    assert got == 0xFF, f"Expected 0xFF got 0x{got:02X}"


@cocotb.test()
async def test_back_to_back(dut):
    """Three bytes sent sequentially; each decoded before starting the next.
    One tx_send task is started at a time so the sender and decoder
    are never racing over the same tx_pin transition."""
    await reset_dut(dut)
    expected = [0x12, 0x34, 0x56]
    for exp in expected:
        cocotb.start_soon(tx_send(dut, exp))
        got = await tx_decode(dut)
        assert got == exp, f"Expected 0x{exp:02X} got 0x{got:02X}"


@cocotb.test()
async def test_tx_ready_low_during_transmit(dut):
    """tx_ready must deassert as soon as tx_valid is accepted."""
    await reset_dut(dut)

    # Wait until idle (tx_ready high)
    while not dut.tx_ready.value:
        await RisingEdge(dut.clk)

    # Start transmission
    dut.tx_data.value  = 0xA5
    dut.tx_valid.value = 1
    await RisingEdge(dut.clk)
    dut.tx_valid.value = 0

    # tx_ready must now be low
    await RisingEdge(dut.clk)
    assert not dut.tx_ready.value, "tx_ready must be low during transmission"

    # Wait for tx_ready to come back high
    for _ in range(CLKS_PER_BIT * 12):
        await RisingEdge(dut.clk)
        if dut.tx_ready.value:
            break
    else:
        raise AssertionError("tx_ready never reasserted after transmission")


@cocotb.test()
async def test_all_256_values(dut):
    """Every possible byte value transmitted correctly."""
    await reset_dut(dut)
    for value in range(256):
        cocotb.start_soon(tx_send(dut, value))
        got = await tx_decode(dut)
        assert got == value, f"Value 0x{value:02X}: got 0x{got:02X}"


@cocotb.test()
async def test_random_bytes(dut):
    """32 random bytes."""
    await reset_dut(dut)
    rng = random.Random(0xC0FFEE)
    for _ in range(32):
        value = rng.randint(0, 255)
        cocotb.start_soon(tx_send(dut, value))
        got = await tx_decode(dut)
        assert got == value, f"Expected 0x{value:02X} got 0x{got:02X}"
