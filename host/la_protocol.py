"""
la_protocol.py — Pure protocol layer for the FPGA Logic Analyzer.

Defines packet building/parsing, command codes, status codes, and
checksum calculation.  No I/O; no serial port dependency.

Packet frame (both directions):
    Byte 0:   0xAA           Magic / start-of-frame
    Byte 1:   CMD            Command code
    Byte 2:   LEN            Payload byte count N (0-8)
    Byte 3…:  PAYLOAD[0..N-1]
    Byte 3+N: CHK            XOR of bytes 1 through 3+N-1

Checksum:
    chk = CMD ^ LEN ^ payload[0] ^ ... ^ payload[N-1]
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Sequence

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
MAGIC: int = 0xAA
MAGIC_END_RLE: int = 0x87   # end-of-RLE-stream marker (second byte after 0xAA)

# Command codes (host → device)
CMD_PING      = 0x01
CMD_SET_REG   = 0x02
CMD_GET_REG   = 0x03
CMD_ARM       = 0x04
CMD_ABORT     = 0x05
CMD_STATUS    = 0x06
CMD_READ_DATA = 0x07

CMD_NAMES = {
    CMD_PING:      "PING",
    CMD_SET_REG:   "SET_REG",
    CMD_GET_REG:   "GET_REG",
    CMD_ARM:       "ARM",
    CMD_ABORT:     "ABORT",
    CMD_STATUS:    "STATUS",
    CMD_READ_DATA: "READ_DATA",
}

# Status / error codes (device → host)
STS_OK            = 0x00
STS_ALREADY_ARMED = 0x01
STS_NOT_COMPLETE  = 0x02
STS_BAD_CMD       = 0x03
STS_BAD_CHECKSUM  = 0x04
STS_BAD_LEN       = 0x05
STS_BAD_REG       = 0x06

STATUS_NAMES = {
    STS_OK:            "OK",
    STS_ALREADY_ARMED: "ERR_ALREADY_ARMED",
    STS_NOT_COMPLETE:  "ERR_NOT_COMPLETE",
    STS_BAD_CMD:       "ERR_BAD_CMD",
    STS_BAD_CHECKSUM:  "ERR_BAD_CHECKSUM",
    STS_BAD_LEN:       "ERR_BAD_LEN",
    STS_BAD_REG:       "ERR_BAD_REG",
}

# Analyzer FSM state codes (in CMD_STATUS response byte 0)
STATE_IDLE         = 0x00
STATE_ARMED        = 0x01
STATE_TRIGGERED    = 0x02
STATE_POST_CAPTURE = 0x03
STATE_COMPLETE     = 0x04
STATE_READOUT      = 0x05

STATE_NAMES = {
    STATE_IDLE:         "IDLE",
    STATE_ARMED:        "ARMED",
    STATE_TRIGGERED:    "TRIGGERED",
    STATE_POST_CAPTURE: "POST_CAPTURE",
    STATE_COMPLETE:     "COMPLETE",
    STATE_READOUT:      "READOUT",
}

# Trigger mode codes (TRIG_MODE register 0x00)
TRIG_IMMEDIATE = 0x00
TRIG_EQUALITY  = 0x01
TRIG_RISING    = 0x02
TRIG_FALLING   = 0x03

TRIG_NAMES = {
    TRIG_IMMEDIATE: "IMMEDIATE",
    TRIG_EQUALITY:  "EQUALITY",
    TRIG_RISING:    "RISING",
    TRIG_FALLING:   "FALLING",
}

TRIG_FROM_NAME = {v: k for k, v in TRIG_NAMES.items()}

# Register addresses (config_regs)
REG_TRIG_MODE       = 0x00
REG_TRIG_MASK       = 0x01
REG_TRIG_VALUE      = 0x02
REG_PRE_TRIG_DEPTH  = 0x03
REG_POST_TRIG_DEPTH = 0x04
REG_RLE_EN          = 0x05
REG_PROBE_MASK      = 0x06


# ---------------------------------------------------------------------------
# Exceptions
# ---------------------------------------------------------------------------
class ProtocolError(Exception):
    """Base class for protocol errors."""


class ChecksumError(ProtocolError):
    """Raised when a received packet has an invalid checksum."""


class BadResponseError(ProtocolError):
    """Raised when a response packet is malformed or unexpected."""


class DeviceError(ProtocolError):
    """Raised when the device returns a non-OK status code."""
    def __init__(self, code: int):
        self.code = code
        super().__init__(f"Device returned error: {STATUS_NAMES.get(code, f'0x{code:02X}')}")


# ---------------------------------------------------------------------------
# Checksum
# ---------------------------------------------------------------------------
def calc_checksum(cmd: int, payload: bytes | Sequence[int]) -> int:
    """Compute XOR checksum over CMD, LEN, and all payload bytes."""
    chk = cmd ^ len(payload)
    for b in payload:
        chk ^= b
    return chk & 0xFF


# ---------------------------------------------------------------------------
# Packet building
# ---------------------------------------------------------------------------
def build_packet(cmd: int, payload: bytes | Sequence[int] = b"") -> bytes:
    """Build a complete framed packet."""
    payload = bytes(payload)
    if len(payload) > 8:
        raise ValueError(f"Payload too long: {len(payload)} bytes (max 8)")
    chk = calc_checksum(cmd, payload)
    return bytes([MAGIC, cmd, len(payload)] + list(payload) + [chk])


# ---------------------------------------------------------------------------
# Packet parsing
# ---------------------------------------------------------------------------
@dataclass
class Packet:
    cmd: int
    payload: bytes

    @property
    def cmd_name(self) -> str:
        return CMD_NAMES.get(self.cmd, f"0x{self.cmd:02X}")


def parse_packet(data: bytes, verify_checksum: bool = True) -> Packet:
    """
    Parse a single framed packet from *data* (must be a complete frame).
    Raises ChecksumError if the checksum is wrong.
    """
    if len(data) < 4:
        raise BadResponseError(f"Packet too short: {len(data)} bytes")
    if data[0] != MAGIC:
        raise BadResponseError(f"Missing magic byte: 0x{data[0]:02X}")
    cmd = data[1]
    length = data[2]
    if len(data) < 4 + length:
        raise BadResponseError(
            f"Truncated packet: expected {4 + length} bytes, got {len(data)}"
        )
    payload = bytes(data[3 : 3 + length])
    received_chk = data[3 + length]
    if verify_checksum:
        expected_chk = calc_checksum(cmd, payload)
        if received_chk != expected_chk:
            raise ChecksumError(
                f"Checksum mismatch: expected 0x{expected_chk:02X}, "
                f"got 0x{received_chk:02X}"
            )
    return Packet(cmd=cmd, payload=payload)


# ---------------------------------------------------------------------------
# Response decoders
# ---------------------------------------------------------------------------
def decode_status_response(payload: bytes) -> dict:
    """
    Decode the payload of a CMD_STATUS response.
    Returns: {'state': int, 'state_name': str, 'trig_position': int, 'rle_en': bool}
    """
    if len(payload) < 4:
        raise BadResponseError(f"STATUS payload too short: {len(payload)}")
    state      = payload[0]
    trig_pos   = (payload[1] << 8) | payload[2]
    rle_en     = bool(payload[3])
    return {
        "state":       state,
        "state_name":  STATE_NAMES.get(state, f"UNKNOWN(0x{state:02X})"),
        "trig_position": trig_pos,
        "rle_en":      rle_en,
    }


def decode_reg_response(payload: bytes) -> int:
    """Decode the 4-byte big-endian value from a CMD_GET_REG response."""
    if len(payload) < 4:
        raise BadResponseError(f"GET_REG payload too short: {len(payload)}")
    return int.from_bytes(payload[:4], "big")


# ---------------------------------------------------------------------------
# RLE decoder
# ---------------------------------------------------------------------------
def rle_decode(data: bytes) -> list[int]:
    """
    Decode an RLE-encoded sample stream back to a list of 16-bit integers.

    Token types:
        Literal (3 bytes):  0x00  hi  lo   → sample = (hi << 8) | lo
        Run     (1 byte):   0x80 | count   → repeat previous sample 'count' times
    """
    samples: list[int] = []
    prev: int | None = None
    i = 0
    while i < len(data):
        b = data[i]
        if b & 0x80:
            # Run token
            count = b & 0x7F
            if prev is None:
                raise ProtocolError("RLE run token before any literal")
            samples.extend([prev] * count)
            i += 1
        else:
            # Literal token
            if i + 2 >= len(data):
                raise ProtocolError(f"Truncated literal at offset {i}")
            val = (data[i + 1] << 8) | data[i + 2]
            samples.append(val)
            prev = val
            i += 3
    return samples


# ---------------------------------------------------------------------------
# Raw sample decoder
# ---------------------------------------------------------------------------
def raw_decode(data: bytes, probe_width: int = 16) -> list[int]:
    """
    Decode a raw sample stream (2 bytes per sample, MSB first).
    Returns a list of integers.
    """
    bytes_per_sample = probe_width // 8
    if len(data) % bytes_per_sample != 0:
        raise ProtocolError(
            f"Raw data length {len(data)} not a multiple of {bytes_per_sample}"
        )
    samples = []
    for i in range(0, len(data), bytes_per_sample):
        val = int.from_bytes(data[i : i + bytes_per_sample], "big")
        samples.append(val)
    return samples
