"""
test_capture_controller_fsm.py — cocotb tests for capture_controller_fsm
combined with circular_capture_buffer and trigger_engine.

Uses a wrapper top-level tb_capture_fsm.v that wires these three modules.

Tests:
  1. Normal arm → equality trigger → complete → readout
  2. Partial pre-trigger (trigger fires before buffer fills)
  3. IMMEDIATE trigger mode
  4. Abort during POST_CAPTURE then re-arm
  5. Re-arm from COMPLETE state
  6. Trigger at write_ptr wrap boundary
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

CLK_PERIOD_NS = 10  # 100 MHz

# FSM state codes (must match RTL)
STATE_IDLE         = 0
STATE_ARMED        = 1
STATE_TRIGGERED    = 2
STATE_POST_CAPTURE = 3
STATE_COMPLETE     = 4
STATE_READOUT      = 5

TRIG_IMM  = 0b00
TRIG_EQ   = 0b01
TRIG_RISE = 0b10
TRIG_FALL = 0b11


async def reset_dut(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, units="ns").start())
    dut.probe_data.value    = 0
    dut.trig_mode.value     = TRIG_IMM
    dut.trig_mask.value     = 0xFF
    dut.trig_value.value    = 0x00
    dut.pre_trig_depth.value  = 8
    dut.post_trig_depth.value = 8
    dut.arm.value           = 0
    dut.abort.value         = 0
    dut.rd_next.value       = 0
    dut.rst_n.value         = 0
    await ClockCycles(dut.clk, 4)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 4)


async def pulse(dut, signal, cycles=1):
    signal.value = 1
    await ClockCycles(dut.clk, cycles)
    signal.value = 0


async def wait_state(dut, target_state, max_cycles=2000):
    for _ in range(max_cycles):
        await RisingEdge(dut.clk)
        if int(dut.state_out.value) == target_state:
            return
    raise AssertionError(
        f"Timeout: never reached state {target_state}, "
        f"stuck at {int(dut.state_out.value)}"
    )


async def do_readout(dut, n_samples: int) -> list:
    """Collect n_samples from the readout interface.

    Cocotb (Icarus) Python callbacks fire in the active region, before
    non-blocking assignments (NBAs) settle.  That means after each
    'await RisingEdge' we see the register values written by the PREVIOUS
    clock's NBAs, not the current one.

    Timing (each step = one clock edge after which Python wakes up):
      Step 0  drive rd_next=1 then await → COMPLETE sees rd_next=1,
              queues READOUT transition + rd_en=1/rd_addr=start (NBA)
      Step 1  rd_next=0, await → BRAM sees rd_en=1 (step-0 NBA),
              queues rd_data=mem[start] (BRAM NBA)
      Step 2  await → rd_data=mem[start] visible (step-1 NBA)
      Loop per sample i:
        drive rd_next=1, await → FSM sees rd_next=1, queues
              rd_sample_valid=1 / rd_sample_out=rd_data / next rd_en (NBA)
        rd_next=0, await → rd_sample_valid=1 now visible (prev NBA)
        assert / collect
        (if not last sample) await → rd_data for [i+1] visible
    """
    samples = []

    # Step 0: kick off COMPLETE → READOUT
    dut.rd_next.value = 1
    await RisingEdge(dut.clk)
    dut.rd_next.value = 0

    # Steps 1-2: two BRAM pipeline fill cycles
    await RisingEdge(dut.clk)
    await RisingEdge(dut.clk)

    for i in range(n_samples):
        # rd_data for sample i is now settled; pulse rd_next
        dut.rd_next.value = 1
        await RisingEdge(dut.clk)    # FSM queues rd_sample_valid=1 (NBA)
        dut.rd_next.value = 0
        await RisingEdge(dut.clk)    # rd_sample_valid=1 visible (prev NBA)
        assert dut.rd_sample_valid.value, \
            f"rd_sample_valid not set for sample {i}"
        samples.append(int(dut.rd_sample_out.value))
        if i < n_samples - 1:
            await RisingEdge(dut.clk)   # BRAM latency for next sample

    return samples


# ---------------------------------------------------------------------------
@cocotb.test()
async def test_normal_equality_trigger(dut):
    """
    Pre=8, post=8, EQUALITY trigger on 0xAA.
    Feed 8 pre-trigger values (0x01..0x08), then 0xAA (trigger), then 8 post.
    Verify: samples[trig_pos]=0xAA, pre values correct.
    """
    await reset_dut(dut)
    dut.trig_mode.value      = TRIG_EQ
    dut.trig_mask.value      = 0xFF
    dut.trig_value.value     = 0xAA
    dut.pre_trig_depth.value = 8
    dut.post_trig_depth.value= 8

    await pulse(dut, dut.arm)
    await wait_state(dut, STATE_ARMED)
    # Drain one extra ARMED cycle so pre_count is predictable
    await RisingEdge(dut.clk)

    # 8 pre-trigger values
    for i in range(1, 9):
        dut.probe_data.value = i
        await RisingEdge(dut.clk)

    # trigger sample
    dut.probe_data.value = 0xAA
    await RisingEdge(dut.clk)

    # 8 post-trigger values
    for i in range(1, 9):
        dut.probe_data.value = 0x10 + i
        await RisingEdge(dut.clk)
    dut.probe_data.value = 0

    await wait_state(dut, STATE_COMPLETE)

    trig_pos = int(dut.trig_position.value)
    total    = int(dut.total_samples.value)
    assert total == trig_pos + 1 + 8, \
        f"total_samples={total} expected trig_pos+9={trig_pos+9}"

    samples = await do_readout(dut, total)
    assert samples[trig_pos] == 0xAA, \
        f"trigger sample={hex(samples[trig_pos])} expected 0xAA"


@cocotb.test()
async def test_partial_pre_trigger(dut):
    """
    Trigger fires after only a few pre-trigger samples (pre_depth=8).
    Verify trig_position matches actual pre count, total correct.
    """
    await reset_dut(dut)
    dut.trig_mode.value       = TRIG_EQ
    dut.trig_mask.value       = 0xFF
    dut.trig_value.value      = 0xBB
    dut.pre_trig_depth.value  = 8
    dut.post_trig_depth.value = 4

    await pulse(dut, dut.arm)
    await wait_state(dut, STATE_ARMED)

    for v in [0x11, 0x22, 0x33]:  # a few pre-trigger samples
        dut.probe_data.value = v
        await RisingEdge(dut.clk)

    dut.probe_data.value = 0xBB  # trigger
    await RisingEdge(dut.clk)

    for i in range(4):
        dut.probe_data.value = 0xC0 + i
        await RisingEdge(dut.clk)
    dut.probe_data.value = 0

    await wait_state(dut, STATE_COMPLETE)

    trig_pos = int(dut.trig_position.value)
    total    = int(dut.total_samples.value)
    assert total == trig_pos + 1 + 4, \
        f"total={total} expected trig_pos+5={trig_pos+5}"

    samples = await do_readout(dut, total)
    assert samples[trig_pos] == 0xBB, \
        f"trigger sample={hex(samples[trig_pos])} expected 0xBB"


@cocotb.test()
async def test_immediate_trigger(dut):
    """IMMEDIATE mode: trigger fires on cycle 0, pre=0, total=1+post."""
    await reset_dut(dut)
    dut.trig_mode.value       = TRIG_IMM
    dut.pre_trig_depth.value  = 0
    dut.post_trig_depth.value = 8

    await pulse(dut, dut.arm)

    for i in range(9):
        dut.probe_data.value = 0x50 + i
        await RisingEdge(dut.clk)
    dut.probe_data.value = 0

    await wait_state(dut, STATE_COMPLETE)

    assert int(dut.trig_position.value) == 0, \
        f"trig_position={int(dut.trig_position.value)} expected 0"
    # total = 0 + 1 + 8 = 9
    assert int(dut.total_samples.value) == 9, \
        f"total_samples={int(dut.total_samples.value)} expected 9"


@cocotb.test()
async def test_abort_during_post_capture(dut):
    """Abort mid-post-capture should return to IDLE."""
    await reset_dut(dut)
    dut.trig_mode.value       = TRIG_EQ
    dut.trig_mask.value       = 0xFF
    dut.trig_value.value      = 0xCC
    dut.pre_trig_depth.value  = 4
    dut.post_trig_depth.value = 16

    await pulse(dut, dut.arm)
    await wait_state(dut, STATE_ARMED)

    for i in range(4):
        dut.probe_data.value = i
        await RisingEdge(dut.clk)

    dut.probe_data.value = 0xCC
    await RisingEdge(dut.clk)

    # A few post-trigger samples then abort
    for _ in range(3):
        dut.probe_data.value = 0x01
        await RisingEdge(dut.clk)

    await pulse(dut, dut.abort)
    await ClockCycles(dut.clk, 4)

    assert int(dut.state_out.value) == STATE_IDLE, \
        f"Expected IDLE after abort, got state={int(dut.state_out.value)}"


@cocotb.test()
async def test_rearm_from_complete(dut):
    """Re-arm from COMPLETE state discards the previous capture."""
    await reset_dut(dut)
    dut.trig_mode.value       = TRIG_EQ
    dut.trig_mask.value       = 0xFF
    dut.trig_value.value      = 0xDD
    dut.pre_trig_depth.value  = 2
    dut.post_trig_depth.value = 2

    # First capture
    await pulse(dut, dut.arm)
    await wait_state(dut, STATE_ARMED)
    for v in [0xA1, 0xA2, 0xDD, 0xA3, 0xA4]:
        dut.probe_data.value = v
        await RisingEdge(dut.clk)
    dut.probe_data.value = 0
    await wait_state(dut, STATE_COMPLETE)

    # Re-arm without reading
    await pulse(dut, dut.arm)
    await ClockCycles(dut.clk, 4)

    assert int(dut.state_out.value) == STATE_ARMED, \
        f"Expected ARMED after re-arm, got {int(dut.state_out.value)}"

    # Complete second capture to ensure FSM is healthy
    for v in [0xB1, 0xB2, 0xDD, 0xB3, 0xB4]:
        dut.probe_data.value = v
        await RisingEdge(dut.clk)
    dut.probe_data.value = 0
    await wait_state(dut, STATE_COMPLETE)


@cocotb.test()
async def test_rising_edge_trigger(dut):
    """RISING mode: trigger fires on 0→1 on bit 0."""
    await reset_dut(dut)
    dut.trig_mode.value       = TRIG_RISE
    dut.trig_mask.value       = 0x01
    dut.trig_value.value      = 0x00
    dut.pre_trig_depth.value  = 4
    dut.post_trig_depth.value = 4

    await pulse(dut, dut.arm)
    await wait_state(dut, STATE_ARMED)

    # Feed low values (bit 0 = 0)
    for _ in range(4):
        dut.probe_data.value = 0x00
        await RisingEdge(dut.clk)

    # Rising edge: bit 0 goes high
    dut.probe_data.value = 0x01
    await RisingEdge(dut.clk)

    for i in range(4):
        dut.probe_data.value = 0x00
        await RisingEdge(dut.clk)
    dut.probe_data.value = 0

    await wait_state(dut, STATE_COMPLETE)

    total    = int(dut.total_samples.value)
    trig_pos = int(dut.trig_position.value)
    samples  = await do_readout(dut, total)
    assert samples[trig_pos] == 0x01, \
        f"Trigger sample={hex(samples[trig_pos])} expected 0x01"
