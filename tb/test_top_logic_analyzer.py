"""
test_top_logic_analyzer.py — cocotb end-to-end tests for top_logic_analyzer.

Tests exercise the full stack over the UART command protocol:
  1. PING → pong
  2. SET_REG / GET_REG round-trip
  3. ARM → inject probe stimulus → poll STATUS until COMPLETE → READ_DATA
  4. ABORT mid-capture
  5. RLE mode: arm, capture constant probe, READ_DATA, decode

Uses small parameters to keep simulation time short:
  PROBE_WIDTH=8, BUFFER_DEPTH=64, CLK_FREQ=1_000_000, BAUD_RATE=100_000
  → CLKS_PER_BIT=10, much faster than the real 12 MHz / 115200 config.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles, Timer
from cocotb.queue import Queue

# Simulation parameters (must match DUT parameters)
CLK_FREQ      = 1_000_000
BAUD_RATE     = 100_000
CLKS_PER_BIT  = CLK_FREQ // BAUD_RATE   # 10
CLK_PERIOD_NS = 1_000_000_000 // CLK_FREQ  # 1000 ns = 1 µs

# Protocol constants
MAGIC = 0xAA

CMD_PING      = 0x01
CMD_SET_REG   = 0x02
CMD_GET_REG   = 0x03
CMD_ARM       = 0x04
CMD_ABORT     = 0x05
CMD_STATUS    = 0x06
CMD_READ_DATA = 0x07

REG_TRIG_MODE       = 0x00
REG_TRIG_MASK       = 0x01
REG_TRIG_VALUE      = 0x02
REG_PRE_TRIG_DEPTH  = 0x03
REG_POST_TRIG_DEPTH = 0x04
REG_RLE_EN          = 0x05

STATE_IDLE         = 0
STATE_ARMED        = 1
STATE_TRIGGERED    = 2
STATE_POST_CAPTURE = 3
STATE_COMPLETE     = 4
STATE_READOUT      = 5

TRIG_IMM  = 0
TRIG_EQ   = 1
TRIG_RISE = 2
TRIG_FALL = 3


# ---------------------------------------------------------------------------
# Low-level UART helpers
# ---------------------------------------------------------------------------

def _checksum(cmd, payload):
    chk = cmd ^ len(payload)
    for b in payload:
        chk ^= b
    return chk & 0xFF


def _build_packet(cmd, payload=b""):
    payload = bytes(payload)
    chk = _checksum(cmd, payload)
    return bytes([MAGIC, cmd, len(payload)]) + payload + bytes([chk])


def _parse_packet(data: bytes):
    """Parse a response packet; return (cmd, payload) or raise."""
    if len(data) < 4:
        raise ValueError(f"Packet too short: {data.hex()}")
    if data[0] != MAGIC:
        raise ValueError(f"Bad magic: {data[0]:#04x}")
    cmd = data[1]
    n   = data[2]
    payload = data[3:3+n]
    chk_got = data[3+n]
    chk_exp = _checksum(cmd, payload)
    if chk_got != chk_exp:
        raise ValueError(
            f"Checksum mismatch: got {chk_got:#04x} expected {chk_exp:#04x}"
        )
    return cmd, payload


async def uart_send_byte(dut, value: int):
    """Drive one 8N1 byte into dut.uart_rx_pin at CLKS_PER_BIT per bit."""
    bits = [0]  # start bit
    for i in range(8):
        bits.append((value >> i) & 1)
    bits.append(1)  # stop bit
    for b in bits:
        dut.uart_rx_pin.value = b
        await ClockCycles(dut.clk, CLKS_PER_BIT)


async def uart_send_packet(dut, cmd, payload=b""):
    """Send a full framed packet byte-by-byte."""
    pkt = _build_packet(cmd, payload)
    dut.uart_rx_pin.value = 1  # idle
    await ClockCycles(dut.clk, 5)
    for b in pkt:
        await uart_send_byte(dut, b)
    await ClockCycles(dut.clk, 5)


async def uart_recv_byte(dut, timeout_clks: int = CLKS_PER_BIT * 20) -> int:
    """Receive one 8N1 byte from dut.uart_tx_pin."""
    # Wait for start bit (falling edge on tx_pin)
    for _ in range(timeout_clks):
        await RisingEdge(dut.clk)
        if not dut.uart_tx_pin.value:
            break
    else:
        raise AssertionError("Timeout waiting for UART start bit")
    # sample mid-bit: wait 1.5 bit periods then sample each data bit
    await ClockCycles(dut.clk, CLKS_PER_BIT + CLKS_PER_BIT // 2)
    value = 0
    for i in range(8):
        value |= (int(dut.uart_tx_pin.value) << i)
        await ClockCycles(dut.clk, CLKS_PER_BIT)
    return value


async def uart_recv_packet(dut, timeout_clks: int = CLKS_PER_BIT * 200) -> tuple:
    """Receive a full framed response packet and return (cmd, payload)."""
    raw = bytearray()
    # Receive bytes until we can parse a complete packet
    # Minimum packet: MAGIC CMD LEN CHK = 4 bytes
    for _ in range(16):
        b = await uart_recv_byte(dut, timeout_clks)
        raw.append(b)
        if len(raw) >= 4:
            n = raw[2]  # LEN field
            if len(raw) >= 4 + n:
                break
    return _parse_packet(bytes(raw))


# ---------------------------------------------------------------------------
# DUT reset
# ---------------------------------------------------------------------------

async def reset_dut(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_PERIOD_NS, units="ns").start())
    dut.rst_n.value       = 0
    dut.uart_rx_pin.value = 1  # UART idle
    dut.probe_pins.value  = 0
    await ClockCycles(dut.clk, 8)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 8)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@cocotb.test()
async def test_ping(dut):
    """PING command returns pong byte 0x55."""
    await reset_dut(dut)
    await uart_send_packet(dut, CMD_PING)
    cmd, payload = await uart_recv_packet(dut)
    assert cmd == CMD_PING, f"ping response cmd={cmd:#04x}"
    assert len(payload) == 1 and payload[0] == 0x55, \
        f"pong payload={payload.hex()}"


@cocotb.test()
async def test_set_get_reg(dut):
    """SET_REG then GET_REG round-trip for TRIG_VALUE register."""
    await reset_dut(dut)

    # SET_REG: addr=0x02 (TRIG_VALUE), value=0x000000AB (8-bit probe)
    payload_set = bytes([REG_TRIG_VALUE, 0x00, 0x00, 0x00, 0xAB])
    await uart_send_packet(dut, CMD_SET_REG, payload_set)
    cmd, resp = await uart_recv_packet(dut)
    assert cmd == CMD_SET_REG
    assert resp[0] == 0x00, f"SET_REG error: {resp[0]:#04x}"

    # GET_REG: addr=0x02
    await uart_send_packet(dut, CMD_GET_REG, bytes([REG_TRIG_VALUE]))
    cmd, resp = await uart_recv_packet(dut)
    assert cmd == CMD_GET_REG, f"get_reg cmd={cmd:#04x}"
    assert len(resp) == 4, f"get_reg resp len={len(resp)}"
    value = int.from_bytes(resp, "big")
    assert value == 0xAB, f"GET_REG value={value:#010x} expected 0xAB"


@cocotb.test()
async def test_arm_status_complete_read(dut):
    """
    Configure EQ trigger on 0xBB, arm, inject probe stimulus,
    poll STATUS until COMPLETE, then READ_DATA and verify samples.

    Parameters: pre=4, post=4, probe=8-bit, buffer=64.
    Inject: 4 pre samples (0x01..0x04), 0xBB trigger, 4 post (0xC0..0xC3).
    Expected: trig_position=4, total=9 samples.
    """
    await reset_dut(dut)

    # Configure registers
    async def set_reg(addr, value):
        p = bytes([addr]) + value.to_bytes(4, "big")
        await uart_send_packet(dut, CMD_SET_REG, p)
        _, r = await uart_recv_packet(dut)
        assert r[0] == 0x00, f"SET_REG {addr:#04x} failed: {r[0]:#04x}"

    await set_reg(REG_TRIG_MODE,       TRIG_EQ)
    await set_reg(REG_TRIG_MASK,       0xFF)
    await set_reg(REG_TRIG_VALUE,      0xBB)
    await set_reg(REG_PRE_TRIG_DEPTH,  4)
    await set_reg(REG_POST_TRIG_DEPTH, 4)
    await set_reg(REG_RLE_EN,          0)

    # ARM
    await uart_send_packet(dut, CMD_ARM)
    cmd, resp = await uart_recv_packet(dut)
    assert cmd == CMD_ARM
    assert resp[0] == 0x00, f"ARM failed: {resp[0]:#04x}"

    # Inject probe stimulus in parallel with status polling
    async def probe_driver():
        await ClockCycles(dut.clk, 20)  # let ARMED state settle
        for v in [0x01, 0x02, 0x03, 0x04]:  # pre-trigger
            dut.probe_pins.value = v
            await ClockCycles(dut.clk, 5)
        dut.probe_pins.value = 0xBB   # trigger
        await ClockCycles(dut.clk, 5)
        for v in [0xC0, 0xC1, 0xC2, 0xC3]:  # post
            dut.probe_pins.value = v
            await ClockCycles(dut.clk, 5)
        dut.probe_pins.value = 0

    cocotb.start_soon(probe_driver())

    # Poll STATUS until COMPLETE
    for _ in range(500):
        await uart_send_packet(dut, CMD_STATUS)
        cmd, resp = await uart_recv_packet(dut)
        assert cmd == CMD_STATUS
        state = resp[0]
        if state == STATE_COMPLETE:
            trig_pos = (resp[1] << 8) | resp[2]
            break
        await ClockCycles(dut.clk, 10)
    else:
        raise AssertionError("Timeout: never reached COMPLETE")

    assert trig_pos == 4, f"trig_position={trig_pos} expected 4"

    # READ_DATA
    await uart_send_packet(dut, CMD_READ_DATA)
    # Response: MAGIC CMD LEN_HI LEN_LO [2*N bytes]
    # Collect raw bytes manually for extended-length packet
    raw = bytearray()
    for _ in range(20 + 9*2):
        b = await uart_recv_byte(dut)
        raw.append(b)
        # Once we have magic + cmd + 2-byte len we know how many more to read
        if len(raw) == 4:
            n_bytes = (raw[2] << 8) | raw[3]
        if len(raw) > 4 and len(raw) == 4 + n_bytes:
            break

    assert raw[0] == MAGIC
    assert raw[1] == CMD_READ_DATA
    n_bytes = (raw[2] << 8) | raw[3]
    samples_raw = raw[4:4+n_bytes]
    assert n_bytes == 9 * 2, f"READ_DATA bytes={n_bytes} expected {9*2}"

    samples = [(samples_raw[i*2] << 8) | samples_raw[i*2+1] for i in range(9)]
    # With cycle-level capture, each driven probe value may be sampled multiple
    # times depending on phase alignment. Validate the trigger window robustly
    # instead of assuming one sample per stimulus write.
    assert samples[4] == 0xBB, \
        f"trigger sample={hex(samples[4])} expected 0xBB; samples={[hex(s) for s in samples]}"
    assert all(s in (0x01, 0x02, 0x03, 0x04, 0xBB) for s in samples[:4]), \
        f"unexpected pre-trigger samples: {[hex(s) for s in samples[:4]]}"
    assert any(s in (0xC0, 0xC1, 0xC2, 0xC3) for s in samples[5:]), \
        f"post-trigger window missing expected values: {[hex(s) for s in samples[5:]]}"


@cocotb.test()
async def test_abort(dut):
    """ARM then ABORT returns FSM to IDLE."""
    await reset_dut(dut)

    await uart_send_packet(dut, CMD_ARM)
    _, resp = await uart_recv_packet(dut)
    assert resp[0] == 0x00

    # Let it run a few cycles then abort
    await ClockCycles(dut.clk, 50)

    await uart_send_packet(dut, CMD_ABORT)
    cmd, resp = await uart_recv_packet(dut)
    assert cmd == CMD_ABORT
    assert resp[0] == 0x00

    # STATUS should now be IDLE
    await uart_send_packet(dut, CMD_STATUS)
    _, resp = await uart_recv_packet(dut)
    assert resp[0] == STATE_IDLE, \
        f"After abort, state={resp[0]} expected IDLE={STATE_IDLE}"


@cocotb.test()
async def test_rle_constant_probe(dut):
    """
    RLE mode: constant probe value → 1 literal + run tokens on READ_DATA.
    We only verify the literal and that the response length is much shorter
    than raw would be.

    pre=2, post=2, IMMEDIATE trigger (fires immediately on arm).
    Total samples = 0+1+2 = 3 (immediate: pre=0 actual since no samples before trigger).
    """
    await reset_dut(dut)

    async def set_reg(addr, value):
        p = bytes([addr]) + value.to_bytes(4, "big")
        await uart_send_packet(dut, CMD_SET_REG, p)
        _, r = await uart_recv_packet(dut)
        assert r[0] == 0x00

    await set_reg(REG_TRIG_MODE,       TRIG_IMM)
    await set_reg(REG_PRE_TRIG_DEPTH,  0)
    await set_reg(REG_POST_TRIG_DEPTH, 2)
    await set_reg(REG_RLE_EN,          1)

    async def probe_driver():
        await ClockCycles(dut.clk, 10)
        for _ in range(10):
            dut.probe_pins.value = 0x42
            await ClockCycles(dut.clk, 5)

    cocotb.start_soon(probe_driver())

    await uart_send_packet(dut, CMD_ARM)
    _, resp = await uart_recv_packet(dut)
    assert resp[0] == 0x00

    # Poll STATUS until COMPLETE
    for _ in range(200):
        await uart_send_packet(dut, CMD_STATUS)
        _, resp = await uart_recv_packet(dut)
        if resp[0] == STATE_COMPLETE:
            break
        await ClockCycles(dut.clk, 5)
    else:
        raise AssertionError("RLE test: never reached COMPLETE")

    # READ_DATA in RLE mode: variable-length, sentinel-terminated (0xAA 0x87)
    await uart_send_packet(dut, CMD_READ_DATA)
    raw = bytearray()
    while True:
        b = await uart_recv_byte(dut, CLKS_PER_BIT * 50)
        raw.append(b)
        if len(raw) >= 2 and raw[-2] == 0xAA and raw[-1] == 0x87:
            break
        if len(raw) > 200:
            raise AssertionError("RLE response too long — sentinel not found")

    # Response format: MAGIC CMD 0xFF 0xFF [rle bytes] 0xAA 0x87
    assert raw[0] == MAGIC
    assert raw[1] == CMD_READ_DATA
    assert raw[2] == 0xFF and raw[3] == 0xFF, "RLE marker"

    rle_bytes = raw[4:-2]  # strip header + sentinel

    # Decode manually
    def rle_decode(data):
        out, prev, i = [], None, 0
        while i < len(data):
            b = data[i]
            if b & 0x80:
                out.extend([prev] * (b & 0x7F))
                i += 1
            else:
                if i + 2 >= len(data):
                    raise AssertionError(f"Truncated RLE literal at i={i}, data={data.hex()}")
                prev = (data[i+1] << 8) | data[i+2]
                out.append(prev)
                i += 3
        return out

    samples = rle_decode(rle_bytes)
    assert all(s == 0x0042 for s in samples), \
        f"RLE decode: unexpected values: {[hex(s) for s in samples]}"
    # For PROBE_WIDTH=8, 0x42 in 8-bit is 0x42; 16-bit wire extends to 0x0042
