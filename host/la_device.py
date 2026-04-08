"""
la_device.py — Serial port abstraction for the FPGA Logic Analyzer.

Wraps pyserial with the logic analyzer protocol.  All public methods
are synchronous: they send a command and block until the response arrives
or a timeout expires.

Usage:
    with LogicAnalyzerDevice("/dev/ttyUSB0") as la:
        la.ping()
        la.set_reg(la_protocol.REG_TRIG_MODE, la_protocol.TRIG_RISING)
        la.arm()
        status = la.wait_for_complete(timeout=10.0)
        data = la.read_data()
"""

from __future__ import annotations

import time
import struct
from typing import Optional

import serial  # pyserial

from . import la_protocol as proto
from .la_protocol import (
    MAGIC, MAGIC_END_RLE,
    CMD_PING, CMD_SET_REG, CMD_GET_REG, CMD_ARM, CMD_ABORT,
    CMD_STATUS, CMD_READ_DATA,
    STATE_COMPLETE,
    ProtocolError, ChecksumError, BadResponseError, DeviceError,
    build_packet, parse_packet, decode_status_response, decode_reg_response,
    raw_decode, rle_decode,
)


class LogicAnalyzerDevice:
    """
    High-level interface to the iCEBreaker logic analyzer over UART.

    Parameters
    ----------
    port : str
        Serial port path, e.g. "/dev/ttyUSB0" or "COM3".
    baud : int
        Baud rate (must match firmware default; default 115200).
    timeout : float
        Per-operation read timeout in seconds.
    probe_width : int
        Probe width in bits (must match firmware parameter; default 16).
    """

    def __init__(
        self,
        port: str,
        baud: int = 115_200,
        timeout: float = 5.0,
        probe_width: int = 16,
    ):
        self._port       = port
        self._baud       = baud
        self._timeout    = timeout
        self._probe_width = probe_width
        self._serial: Optional[serial.Serial] = None

    # -----------------------------------------------------------------------
    # Context manager
    # -----------------------------------------------------------------------
    def open(self) -> None:
        self._serial = serial.Serial(
            port=self._port,
            baudrate=self._baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=self._timeout,
        )
        # Some USB-UART bridges glitch or reset the FPGA on open.
        # Give the link a brief settle window and clear stale bytes.
        self._serial.setDTR(False)
        self._serial.setRTS(False)
        time.sleep(0.10)
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()

    def close(self) -> None:
        if self._serial and self._serial.is_open:
            self._serial.close()

    def __enter__(self) -> "LogicAnalyzerDevice":
        self.open()
        return self

    def __exit__(self, *_) -> None:
        self.close()

    # -----------------------------------------------------------------------
    # Low-level send/receive
    # -----------------------------------------------------------------------
    def _send(self, packet: bytes) -> None:
        assert self._serial, "Device not open"
        self._serial.write(packet)
        self._serial.flush()

    def _recv_byte(self) -> int:
        assert self._serial
        b = self._serial.read(1)
        if not b:
            raise TimeoutError("Timeout waiting for response byte")
        return b[0]

    def _recv_packet(self) -> proto.Packet:
        """
        Read one complete response frame from the device.
        Scans for 0xAA magic byte (absorbs any garbage), then reads the rest.
        """
        # Wait for magic
        while True:
            b = self._recv_byte()
            if b == MAGIC:
                break

        cmd = self._recv_byte()
        length = self._recv_byte()

        # Handle extended-length READ_DATA response (LEN field is 2 bytes)
        if cmd == CMD_READ_DATA and length == 0xFF:
            lo = self._recv_byte()
            if lo == 0xFF:
                # RLE sentinel-terminated mode; collect until 0xAA 0x87
                return self._recv_rle_data_packet()
            else:
                raise BadResponseError(f"Unexpected READ_DATA LEN byte: 0xFF 0x{lo:02X}")
        elif cmd == CMD_READ_DATA:
            # Raw mode: LEN field is 2 bytes (LEN_HI already read as 'length')
            len_lo = self._recv_byte()
            data_len = (length << 8) | len_lo
            data = self._serial.read(data_len)
            if len(data) != data_len:
                raise TimeoutError(f"Truncated READ_DATA: got {len(data)}/{data_len} bytes")
            # Firmware raw streaming sends exactly:
            #   0xAA 0x07 LEN_HI LEN_LO <data_len bytes>
            # with no trailing checksum byte.
            return proto.Packet(cmd=CMD_READ_DATA, payload=bytes(data))

        # Normal fixed-length response
        payload_bytes = bytes(self._serial.read(length))
        if len(payload_bytes) != length:
            raise TimeoutError(f"Truncated payload: got {len(payload_bytes)}/{length}")
        chk_byte = self._recv_byte()
        expected = proto.calc_checksum(cmd, payload_bytes)
        if chk_byte != expected:
            raise ChecksumError(
                f"Checksum mismatch: expected 0x{expected:02X}, got 0x{chk_byte:02X}"
            )
        return proto.Packet(cmd=cmd, payload=payload_bytes)

    def _recv_rle_data_packet(self) -> proto.Packet:
        """Read RLE byte stream until 0xAA 0x87 sentinel."""
        buf = bytearray()
        prev = None
        deadline = time.monotonic() + self._timeout * 10  # RLE can take longer
        while time.monotonic() < deadline:
            b = self._recv_byte()
            if prev == MAGIC and b == MAGIC_END_RLE:
                buf.pop()  # remove the 0xAA we already appended
                return proto.Packet(cmd=CMD_READ_DATA, payload=bytes(buf))
            buf.append(b)
            prev = b
        raise TimeoutError("Timeout waiting for RLE end-of-stream marker")

    def send_cmd(self, cmd: int, payload: bytes = b"") -> proto.Packet:
        """Send command packet and return response Packet."""
        self._send(build_packet(cmd, payload))
        return self._recv_packet()

    # -----------------------------------------------------------------------
    # High-level commands
    # -----------------------------------------------------------------------
    def ping(self) -> bool:
        """Send PING; return True if pong received."""
        assert self._serial, "Device not open"
        # Retry briefly to tolerate FPGA reboot-on-open behavior.
        deadline = time.monotonic() + max(self._timeout, 1.5)
        while time.monotonic() < deadline:
            try:
                resp = self.send_cmd(CMD_PING)
                return resp.payload == bytes([0x55])
            except (TimeoutError, ProtocolError, ChecksumError, BadResponseError):
                # Drop garbage and retry with a short backoff.
                self._serial.reset_input_buffer()
                time.sleep(0.05)
        return False

    def set_reg(self, addr: int, value: int) -> None:
        """Write a 32-bit value to a config register."""
        payload = bytes([addr]) + struct.pack(">I", value)
        resp = self.send_cmd(CMD_SET_REG, payload)
        if resp.payload and resp.payload[0] != proto.STS_OK:
            raise DeviceError(resp.payload[0])

    def get_reg(self, addr: int) -> int:
        """Read a 32-bit value from a config register."""
        resp = self.send_cmd(CMD_GET_REG, bytes([addr]))
        return decode_reg_response(resp.payload)

    def arm(self) -> None:
        """Arm the analyzer for acquisition."""
        resp = self.send_cmd(CMD_ARM)
        if resp.payload and resp.payload[0] not in (proto.STS_OK, proto.STS_ALREADY_ARMED):
            raise DeviceError(resp.payload[0])

    def abort(self) -> None:
        """Abort current acquisition and return to IDLE."""
        self.send_cmd(CMD_ABORT)

    def get_status(self) -> dict:
        """
        Query current status.
        Returns dict with keys: state, state_name, trig_position, rle_en
        """
        resp = self.send_cmd(CMD_STATUS)
        return decode_status_response(resp.payload)

    def read_data_raw(self) -> bytes:
        """
        Read captured data (raw bytes).
        Must be called when state == COMPLETE.
        Returns raw bytes (2 bytes per sample, MSB first).
        """
        resp = self.send_cmd(CMD_READ_DATA)
        if resp.cmd == 0xFF:
            raise DeviceError(resp.payload[0] if resp.payload else 0xFF)
        return resp.payload

    def read_data_decoded(self, rle: bool = False) -> list[int]:
        """
        Read and decode captured samples.
        Returns list of integer sample values.
        """
        raw = self.read_data_raw()
        if rle:
            return rle_decode(raw)
        else:
            return raw_decode(raw, self._probe_width)

    def wait_for_complete(self, timeout: float = 30.0, poll_interval: float = 0.05) -> dict:
        """
        Poll CMD_STATUS until capture reaches COMPLETE state.
        Returns the final status dict.
        Raises TimeoutError if capture does not complete within *timeout* seconds.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            status = self.get_status()
            if status["state"] == STATE_COMPLETE:
                return status
            if status["state"] == proto.STATE_IDLE:
                raise ProtocolError("Analyzer went IDLE unexpectedly (aborted?)")
            time.sleep(poll_interval)
        raise TimeoutError(
            f"Capture did not complete within {timeout}s "
            f"(last state: {status.get('state_name', '?')})"
        )

    def reset(self) -> None:
        """Abort any active capture and return to IDLE."""
        self.abort()
