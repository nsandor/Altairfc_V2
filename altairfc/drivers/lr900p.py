"""
MicoAir LR900P radio modem — protocol primitives.

Frame layout:
    [SOF 0xEF][TYPE][0x00][CMD][SEQ][PAYLOAD...][SUM8]

All classes, threads, and serial I/O live in telemetry/transport.py.
This module is pure functions and dataclasses only.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Callable, Optional

# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------

SOF           = 0xEF
TYPE_REQUEST  = 0x01
TYPE_RESPONSE = 0x1E

CMD_HEARTBEAT   = 0x01
CMD_CONFIG_REQ  = 0x02
CMD_CONFIG_RESP = 0x03

SUBBLOCK_READ  = 0x03
SUBBLOCK_WRITE = 0x04

CONFIG_BLOCK_ID = 0x12
FREQ_WORD_LO    = 0xE8
FREQ_WORD_HI    = 0x03

HEARTBEAT_INTERVAL = 0.625

_PACKET_LENGTHS = {
    (TYPE_REQUEST,  CMD_HEARTBEAT):   20,
    (TYPE_RESPONSE, CMD_HEARTBEAT):   20,
    (TYPE_REQUEST,  CMD_CONFIG_REQ):  25,
    (TYPE_RESPONSE, CMD_CONFIG_RESP): 25,
}

DATA_RATE_LOW  = 0x00
DATA_RATE_MID  = 0x01
DATA_RATE_HIGH = 0x02

TX_POWER_LOW  = 0x00
TX_POWER_MID  = 0x01
TX_POWER_HIGH = 0x02


# ---------------------------------------------------------------------------
# Checksum
# ---------------------------------------------------------------------------

def _checksum(data: bytes) -> int:
    return sum(data) & 0xFF


def _verify(packet: bytes) -> bool:
    return len(packet) >= 2 and _checksum(packet[:-1]) == packet[-1]


# ---------------------------------------------------------------------------
# Packet builders
# ---------------------------------------------------------------------------

def build_heartbeat(seq: int, pc_uptime_ms: int, link_flag: int = 0x00) -> bytes:
    body = bytes([
        SOF, TYPE_REQUEST, 0x00, CMD_HEARTBEAT, seq & 0xFF,
        0x0D,
        pc_uptime_ms & 0xFF, (pc_uptime_ms >> 8) & 0xFF,
        0x0E, 0x00, link_flag,
        0x01, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00,
    ])
    return body + bytes([_checksum(body)])


def build_config_read(seq: int) -> bytes:
    body = bytes([
        SOF, TYPE_REQUEST, 0x00, CMD_CONFIG_REQ, seq & 0xFF,
        CONFIG_BLOCK_ID, SUBBLOCK_READ,
    ]) + bytes(17)
    return body + bytes([_checksum(body)])


def build_config_write(seq: int, data_rate: int, tx_power: int, channel: int) -> bytes:
    body = bytes([
        SOF, TYPE_REQUEST, 0x00, CMD_CONFIG_REQ, seq & 0xFF,
        CONFIG_BLOCK_ID, SUBBLOCK_WRITE,
        0x00, 0x00,
        data_rate & 0xFF,
        tx_power  & 0xFF,
        channel   & 0xFF,
        FREQ_WORD_LO, FREQ_WORD_HI,
        0x03, 0x00,
    ]) + bytes(8)
    return body + bytes([_checksum(body)])


# ---------------------------------------------------------------------------
# Parsed response types
# ---------------------------------------------------------------------------

@dataclass
class HeartbeatResponse:
    seq:             int
    radio_uptime_ms: int
    fw_status:       int
    noise:           int
    peer:            bytes

    @property
    def peer_connected(self) -> bool:
        return self.peer != b'\xFF\xFF\xFF\xFF'


@dataclass
class ConfigResponse:
    seq:        int
    data_rate:  int
    tx_power:   int
    channel:    int
    mode_flags: bytes
    session_id: bytes
    raw:        bytes


@dataclass
class WriteAckResponse:
    seq: int
    raw: bytes


# ---------------------------------------------------------------------------
# Frame reassembler
# ---------------------------------------------------------------------------

class FrameAssembler:
    """Stateful reassembler — feed() it raw bytes; it fires callback per valid frame."""

    def __init__(self, callback: Callable[[bytes], None]):
        self._cb  = callback
        self._buf = bytearray()

    def feed(self, data: bytes) -> None:
        self._buf.extend(data)
        self._process()

    def _process(self) -> None:
        buf = self._buf
        while True:
            sof_idx = buf.find(SOF)
            if sof_idx < 0:
                self._buf = bytearray()
                return
            if sof_idx > 0:
                self._buf = buf[sof_idx:]
                buf = self._buf

            if len(buf) < 6:
                break

            expected = _PACKET_LENGTHS.get((buf[1], buf[3]))
            if expected is None:
                self._buf = buf[1:]
                buf = self._buf
                continue
            if len(buf) < expected:
                break

            raw = bytes(buf[:expected])
            self._buf = buf[expected:]
            buf = self._buf
            if _verify(raw):
                self._cb(raw)


# ---------------------------------------------------------------------------
# Parsers
# ---------------------------------------------------------------------------

def parse_heartbeat_response(raw: bytes) -> Optional[HeartbeatResponse]:
    if len(raw) != 20 or raw[1] != TYPE_RESPONSE or raw[3] != CMD_HEARTBEAT:
        return None
    p = raw[5:-1]
    return HeartbeatResponse(
        seq=raw[4],
        radio_uptime_ms=struct.unpack_from('<H', p, 1)[0],
        fw_status=p[5],
        noise=p[6],
        peer=bytes(p[10:14]),
    )


def parse_config_response(raw: bytes) -> Optional[ConfigResponse]:
    if len(raw) != 25 or raw[1] != TYPE_RESPONSE or raw[3] != CMD_CONFIG_RESP or raw[6] != SUBBLOCK_READ:
        return None
    p = raw[5:-1]
    return ConfigResponse(
        seq=raw[4],
        data_rate=p[4],
        tx_power=p[5],
        channel=p[6],
        mode_flags=bytes(p[12:15]),
        session_id=bytes(p[15:19]),
        raw=raw,
    )


def parse_write_ack(raw: bytes) -> Optional[WriteAckResponse]:
    if len(raw) != 25 or raw[1] != TYPE_RESPONSE or raw[3] != CMD_CONFIG_RESP or raw[6] != SUBBLOCK_WRITE:
        return None
    return WriteAckResponse(seq=raw[4], raw=raw)
