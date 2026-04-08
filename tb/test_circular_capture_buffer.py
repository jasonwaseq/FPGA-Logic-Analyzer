"""
test_circular_capture_buffer.py — cocotb tests for circular_capture_buffer.

Tests: sequential fill, overwrite, wrap-boundary, concurrent read/write.
Note: BRAM read has 1-cycle registered latency — tests account for this.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

CLK_PERIOD_NS = 10  # 100 MHz for fast sim
DEPTH = 64
WIDTH = 16


async def bram_write(dut, addr: int, data: int):
    await RisingEdge(dut.clk)
    dut.wr_addr.value = addr
    dut.wr_data.value = data
    dut.wr_en.value   = 1
    await RisingEdge(dut.clk)
    dut.wr_en.value   = 0


async def bram_read(dut, addr: int) -> int:
    """Issue read; data appears one cycle after address is presented."""
    await RisingEdge(dut.clk)
    dut.rd_addr.value = addr
    dut.rd_en.value   = 1
    await RisingEdge(dut.clk)   # data captured in BRAM output reg
    dut.rd_en.value   = 0
    await RisingEdge(dut.clk)   # sample after next edge
    return int(dut.rd_data.value)


async def reset_dut(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, units="ns").start())
    dut.wr_addr.value = 0
    dut.wr_data.value = 0
    dut.wr_en.value   = 0
    dut.rd_addr.value = 0
    dut.rd_en.value   = 0
    await ClockCycles(dut.clk, 4)


@cocotb.test()
async def test_sequential_fill_and_read(dut):
    """Write DEPTH unique values then read them back."""
    await reset_dut(dut)
    for i in range(DEPTH):
        await bram_write(dut, i, 0xA000 + i)
    for i in range(DEPTH):
        got = await bram_read(dut, i)
        assert got == (0xA000 + i) & 0xFFFF, f"entry[{i}]: got 0x{got:04X}"


@cocotb.test()
async def test_overwrite(dut):
    """Second pass of writes should overwrite first."""
    await reset_dut(dut)
    for i in range(DEPTH):
        await bram_write(dut, i, 0xA000 + i)
    for i in range(DEPTH):
        await bram_write(dut, i, 0xB000 + (DEPTH - 1 - i))
    for i in range(DEPTH):
        got = await bram_read(dut, i)
        expected = (0xB000 + (DEPTH - 1 - i)) & 0xFFFF
        assert got == expected, f"overwrite[{i}]: got 0x{got:04X} expected 0x{expected:04X}"


@cocotb.test()
async def test_wrap_boundary(dut):
    """Write at addresses 0 and DEPTH-1 (circular boundary)."""
    await reset_dut(dut)
    await bram_write(dut, DEPTH - 1, 0xDEAD)
    await bram_write(dut, 0, 0xBEEF)
    got_end = await bram_read(dut, DEPTH - 1)
    got_beg = await bram_read(dut, 0)
    assert got_end == 0xDEAD, f"DEPTH-1: got 0x{got_end:04X}"
    assert got_beg == 0xBEEF, f"addr 0:  got 0x{got_beg:04X}"


@cocotb.test()
async def test_concurrent_write_read_different_addresses(dut):
    """Write addr=10 new value while simultaneously reading addr=20."""
    await reset_dut(dut)
    await bram_write(dut, 10, 0x1111)
    await bram_write(dut, 20, 0x2222)

    # Issue new write to addr=10 and read from addr=20 in same cycle
    await RisingEdge(dut.clk)
    dut.wr_addr.value = 10
    dut.wr_data.value = 0x3333
    dut.wr_en.value   = 1
    dut.rd_addr.value = 20
    dut.rd_en.value   = 1
    await RisingEdge(dut.clk)
    dut.wr_en.value = 0
    dut.rd_en.value = 0
    await RisingEdge(dut.clk)  # data stable

    assert int(dut.rd_data.value) == 0x2222, \
        f"Concurrent read addr=20: got 0x{int(dut.rd_data.value):04X}"

    # Verify write to addr=10 also took effect
    got = await bram_read(dut, 10)
    assert got == 0x3333, f"Concurrent write addr=10: got 0x{got:04X}"
