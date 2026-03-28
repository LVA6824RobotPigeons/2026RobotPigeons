"""BCNP packet framing helpers shared by coprocessor runtime and tests."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import struct
from typing import Mapping
import zlib


PROTOCOL_MAJOR = 3
PROTOCOL_MINOR = 4

HEADER_SIZE = 7
CRC_SIZE = 4
HANDSHAKE_SIZE = 8

FLAG_CLEAR_QUEUE = 0x01

MSG_AUTO_PLAN_REQUEST = 2001
MSG_AUTO_PLAN_RESPONSE = 2002
MSG_AUTO_PHASE_COMMAND = 2003
MSG_AUTO_PHASE_ACK = 2004
MSG_AUTO_HEARTBEAT = 2005
MSG_AUTO_ABORT = 2006
MSG_AUTO_TELEMETRY = 2007
MSG_AUTO_SHOT_HINT = 2008
MSG_AUTO_WORLD_UPDATE = 2009
MSG_AUTO_WAYPOINT_DELTA = 2010
MSG_AUTO_OPPONENT_UPDATE = 2011

WIRE_SIZES = {
    MSG_AUTO_PLAN_REQUEST: 16,
    MSG_AUTO_PLAN_RESPONSE: 20,
    MSG_AUTO_PHASE_COMMAND: 32,
    MSG_AUTO_PHASE_ACK: 12,
    MSG_AUTO_HEARTBEAT: 12,
    MSG_AUTO_ABORT: 8,
    MSG_AUTO_TELEMETRY: 32,
    MSG_AUTO_SHOT_HINT: 16,
    MSG_AUTO_WORLD_UPDATE: 22,
    MSG_AUTO_WAYPOINT_DELTA: 20,
    MSG_AUTO_OPPONENT_UPDATE: 18,
}


class DecodeError(Enum):
    NONE = "NONE"
    INCOMPLETE = "INCOMPLETE"
    UNSUPPORTED_VERSION = "UNSUPPORTED_VERSION"
    UNKNOWN_MESSAGE_TYPE = "UNKNOWN_MESSAGE_TYPE"
    CRC_MISMATCH = "CRC_MISMATCH"


@dataclass(frozen=True)
class DecodedPacket:
    error: DecodeError
    consumed_bytes: int
    message_type: int = 0
    message_count: int = 0
    flags: int = 0
    payload: bytes = b""

    @property
    def is_ok(self) -> bool:
        return self.error == DecodeError.NONE


def build_handshake(schema_hash: int) -> bytes:
    return b"BCNP" + struct.pack(">I", schema_hash & 0xFFFFFFFF)


def is_handshake_magic(data: bytes) -> bool:
    return len(data) >= HANDSHAKE_SIZE and data[:4] == b"BCNP"


def read_handshake_schema_hash(data: bytes) -> int:
    return struct.unpack(">I", data[4:8])[0]


def wire_size(message_type: int, wire_sizes: Mapping[int, int] | None = None) -> int:
    lookup = wire_sizes if wire_sizes is not None else WIRE_SIZES
    return int(lookup.get(message_type, 0))


def encode_packet(message_type: int, flags: int, message_count: int, payload: bytes) -> bytes:
    header = struct.pack(
        ">BBBHH",
        PROTOCOL_MAJOR,
        PROTOCOL_MINOR,
        flags & 0xFF,
        message_type & 0xFFFF,
        message_count & 0xFFFF,
    )
    frame_without_crc = header + payload
    crc = zlib.crc32(frame_without_crc) & 0xFFFFFFFF
    return frame_without_crc + struct.pack(">I", crc)


def decode_packet(
    buffer: bytes,
    *,
    offset: int = 0,
    length: int | None = None,
    wire_sizes: Mapping[int, int] | None = None,
) -> DecodedPacket:
    frame_length = len(buffer) - offset if length is None else length
    if frame_length < HEADER_SIZE + CRC_SIZE:
        return DecodedPacket(DecodeError.INCOMPLETE, 0)

    major, minor, flags, message_type, message_count = struct.unpack_from(">BBBHH", buffer, offset)
    if major != PROTOCOL_MAJOR or minor != PROTOCOL_MINOR:
        return DecodedPacket(DecodeError.UNSUPPORTED_VERSION, 1)

    message_wire_size = wire_size(message_type, wire_sizes)
    if message_wire_size <= 0:
        return DecodedPacket(DecodeError.UNKNOWN_MESSAGE_TYPE, 1)

    payload_length = message_wire_size * message_count
    total_length = HEADER_SIZE + payload_length + CRC_SIZE
    if frame_length < total_length:
        return DecodedPacket(DecodeError.INCOMPLETE, 0)

    expected_crc = zlib.crc32(buffer[offset : offset + HEADER_SIZE + payload_length]) & 0xFFFFFFFF
    packet_crc = struct.unpack_from(">I", buffer, offset + HEADER_SIZE + payload_length)[0]
    if expected_crc != packet_crc:
        return DecodedPacket(DecodeError.CRC_MISMATCH, total_length)

    payload_start = offset + HEADER_SIZE
    payload = bytes(buffer[payload_start : payload_start + payload_length])
    return DecodedPacket(
        DecodeError.NONE,
        consumed_bytes=total_length,
        message_type=message_type,
        message_count=message_count,
        flags=flags,
        payload=payload,
    )
