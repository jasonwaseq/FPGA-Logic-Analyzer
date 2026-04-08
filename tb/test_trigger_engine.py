"""
test_trigger_engine.py — cocotb tests for trigger_engine module.

All tests are pure combinational: set inputs, wait one delta, check output.
Tests: all 4 modes, zero mask, full mask, partial mask, edge cases.
"""

import cocotb
from cocotb.triggers import Timer

W = 16  # PROBE_WIDTH

IMM  = 0b00
EQ   = 0b01
RISE = 0b10
FALL = 0b11


def set_inputs(dut, probe, prev, mode, mask=0xFFFF, value=0x0000):
    dut.probe_data.value = probe
    dut.prev_data.value  = prev
    dut.trig_mode.value  = mode
    dut.trig_mask.value  = mask
    dut.trig_value.value = value


async def check(dut, expected: bool, msg: str):
    await Timer(1, units="ns")  # let combinational settle
    got = bool(dut.trigger_fire.value)
    assert got == expected, f"{msg}: expected {expected}, got {got} " \
        f"(probe=0x{int(dut.probe_data.value):04X} prev=0x{int(dut.prev_data.value):04X} " \
        f"mode={int(dut.trig_mode.value)} mask=0x{int(dut.trig_mask.value):04X} " \
        f"val=0x{int(dut.trig_value.value):04X})"


# ---------------------------------------------------------------------------
# IMMEDIATE mode
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_immediate_always_fires(dut):
    """IMMEDIATE mode fires regardless of probe value."""
    for probe in [0x0000, 0xFFFF, 0x1234, 0xABCD]:
        set_inputs(dut, probe, 0x0000, IMM)
        await check(dut, True, f"IMM probe=0x{probe:04X}")


# ---------------------------------------------------------------------------
# EQUALITY mode
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_equality_exact_match(dut):
    set_inputs(dut, 0xBEEF, 0x0000, EQ, mask=0xFFFF, value=0xBEEF)
    await check(dut, True, "EQ exact match")


@cocotb.test()
async def test_equality_near_miss(dut):
    set_inputs(dut, 0xBEEE, 0x0000, EQ, mask=0xFFFF, value=0xBEEF)
    await check(dut, False, "EQ near miss")


@cocotb.test()
async def test_equality_partial_mask_match(dut):
    """Only low byte in mask — high byte must be ignored."""
    set_inputs(dut, 0xFFBB, 0x0000, EQ, mask=0x00FF, value=0xAABB)
    await check(dut, True, "EQ partial mask match (hi ignored)")


@cocotb.test()
async def test_equality_partial_mask_miss(dut):
    set_inputs(dut, 0xFFAA, 0x0000, EQ, mask=0x00FF, value=0xAABB)
    await check(dut, False, "EQ partial mask miss")


@cocotb.test()
async def test_equality_zero_mask_always_matches(dut):
    """Zero mask: all bits ignored, always matches."""
    for probe in [0x0000, 0xDEAD, 0xFFFF]:
        set_inputs(dut, probe, 0x0000, EQ, mask=0x0000, value=0xDEAD)
        await check(dut, True, f"EQ zero mask probe=0x{probe:04X}")


# ---------------------------------------------------------------------------
# RISING EDGE mode
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_rising_bit0(dut):
    """Bit 0: 0→1 is a rising edge."""
    set_inputs(dut, 0x0001, 0x0000, RISE, mask=0xFFFF)
    await check(dut, True, "RISE bit0 0→1")


@cocotb.test()
async def test_rising_no_edge_stays_high(dut):
    set_inputs(dut, 0x0001, 0x0001, RISE, mask=0xFFFF)
    await check(dut, False, "RISE bit0 steady high")


@cocotb.test()
async def test_rising_falling_is_not_rising(dut):
    set_inputs(dut, 0x0000, 0x0001, RISE, mask=0xFFFF)
    await check(dut, False, "RISE bit0 1→0 is not rising")


@cocotb.test()
async def test_rising_all_bits(dut):
    set_inputs(dut, 0xFFFF, 0x0000, RISE, mask=0xFFFF)
    await check(dut, True, "RISE all bits 0→1")


@cocotb.test()
async def test_rising_masked_bit(dut):
    """Only bit 7 in mask; unmasked bits rising shouldn't fire."""
    set_inputs(dut, 0x0080, 0x0000, RISE, mask=0x0080)
    await check(dut, True, "RISE masked bit7 0→1")


@cocotb.test()
async def test_rising_unmasked_bit_ignored(dut):
    """Unmasked bits rising must NOT fire the trigger."""
    set_inputs(dut, 0xFF7F, 0x0000, RISE, mask=0x0080)
    await check(dut, False, "RISE unmasked bits only")


# ---------------------------------------------------------------------------
# FALLING EDGE mode
# ---------------------------------------------------------------------------
@cocotb.test()
async def test_falling_bit0(dut):
    set_inputs(dut, 0x0000, 0x0001, FALL, mask=0xFFFF)
    await check(dut, True, "FALL bit0 1→0")


@cocotb.test()
async def test_falling_no_edge_stays_low(dut):
    set_inputs(dut, 0x0000, 0x0000, FALL, mask=0xFFFF)
    await check(dut, False, "FALL bit0 steady low")


@cocotb.test()
async def test_falling_rising_is_not_falling(dut):
    set_inputs(dut, 0x0001, 0x0000, FALL, mask=0xFFFF)
    await check(dut, False, "FALL bit0 0→1 is not falling")


@cocotb.test()
async def test_falling_all_bits(dut):
    set_inputs(dut, 0x0000, 0xFFFF, FALL, mask=0xFFFF)
    await check(dut, True, "FALL all bits 1→0")


@cocotb.test()
async def test_falling_masked_bit15(dut):
    set_inputs(dut, 0x0000, 0x8000, FALL, mask=0x8000)
    await check(dut, True, "FALL masked bit15 1→0")


@cocotb.test()
async def test_falling_unmasked_bit_ignored(dut):
    set_inputs(dut, 0x0000, 0x7FFF, FALL, mask=0x8000)
    await check(dut, False, "FALL unmasked bits only")
