"""DCP v1 binary frame codec shared by the car Raspberry Pi gateway."""

from __future__ import annotations

from dataclasses import dataclass
import struct


MAGIC = 0xAA
TAIL = 0xFF
VERSION = 0x01
MAX_PAYLOAD = 256

UAV = 0x01
CAR = 0x02
GROUND = 0x03
BROADCAST = 0xFF

HEARTBEAT = 0x01
UAV_READY = 0x02
CAR_START = 0x03
ACK = 0x04
CAR_STATE = 0x10
UAV_STATE = 0x11
FUSED_POSITION = 0x12
CAR_POSITION = 0x13
# 无人机事件类型（每个是独立 message_type，见协议第72-76行）
DROP_RELEASED = 0x20
TOUCHDOWN_CONFIRMED = 0x21
RETAKEOFF_STARTED = 0x22
MISSION_COMPLETE = 0x23
FAULT_EVENT = 0x30
# 兼容旧代码：UAV_EVENT 指代"任意无人机事件"，实际处理按具体类型
UAV_EVENT = 0x20
# 需要 ACK 的事件集合
EVENTS_REQUIRING_ACK = (DROP_RELEASED, TOUCHDOWN_CONFIRMED, MISSION_COMPLETE)

ACK_REQUIRED = 0x01
IS_ACK = 0x02
EVENT = 0x04
ERROR = 0x08

HEADER_STRUCT = struct.Struct("<BBBBBIHIH")
MIN_FRAME_SIZE = 1 + HEADER_STRUCT.size + 2 + 1


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    """CRC-CCITT-FALSE: polynomial 0x1021, initial value 0xFFFF."""
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class DcpFrame:
    message_type: int
    flags: int
    source: int
    destination: int
    session_id: int
    seq: int
    sender_ms: int
    payload: bytes
    raw: bytes


def build_frame(
    message_type: int,
    payload: bytes,
    *,
    flags: int,
    source: int,
    destination: int,
    session_id: int,
    seq: int,
    sender_ms: int,
) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload too long: {len(payload)}")
    header = HEADER_STRUCT.pack(
        VERSION,
        message_type,
        flags,
        source,
        destination,
        session_id & 0xFFFFFFFF,
        seq & 0xFFFF,
        sender_ms & 0xFFFFFFFF,
        len(payload),
    )
    crc = crc16_ccitt(header + payload)
    return bytes((MAGIC,)) + header + payload + struct.pack("<H", crc) + bytes((TAIL,))


class DcpStreamParser:
    """Length-driven parser that recovers by searching for the next magic byte."""

    def __init__(self) -> None:
        self._buffer = bytearray()
        self.bad_crc = 0
        self.bad_format = 0

    def feed(self, incoming: bytes) -> list[DcpFrame]:
        self._buffer.extend(incoming)
        frames: list[DcpFrame] = []
        while True:
            magic_index = self._buffer.find(MAGIC)
            if magic_index < 0:
                self._buffer.clear()
                break
            if magic_index:
                del self._buffer[:magic_index]
            if len(self._buffer) < MIN_FRAME_SIZE:
                break

            version, msg_type, flags, source, dest, session_id, seq, sender_ms, payload_len = (
                HEADER_STRUCT.unpack_from(self._buffer, 1)
            )
            if version != VERSION or payload_len > MAX_PAYLOAD:
                self.bad_format += 1
                del self._buffer[0]
                continue

            frame_size = 1 + HEADER_STRUCT.size + payload_len + 2 + 1
            if len(self._buffer) < frame_size:
                break
            if self._buffer[frame_size - 1] != TAIL:
                self.bad_format += 1
                del self._buffer[0]
                continue

            payload_start = 1 + HEADER_STRUCT.size
            payload_end = payload_start + payload_len
            received_crc = struct.unpack_from("<H", self._buffer, payload_end)[0]
            actual_crc = crc16_ccitt(bytes(self._buffer[1:payload_end]))
            if received_crc != actual_crc:
                self.bad_crc += 1
                del self._buffer[0]
                continue

            raw = bytes(self._buffer[:frame_size])
            frames.append(
                DcpFrame(
                    msg_type,
                    flags,
                    source,
                    dest,
                    session_id,
                    seq,
                    sender_ms,
                    bytes(self._buffer[payload_start:payload_end]),
                    raw,
                )
            )
            del self._buffer[:frame_size]
        return frames


def seq_is_new(new: int, old: int) -> bool:
    delta = (new - old) & 0xFFFF
    return 0 < delta < 0x8000
