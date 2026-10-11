"""Pi端VS1解析与Cyber Camera重启后的安全重同步。"""

from __future__ import annotations

import binascii
import time
from dataclasses import dataclass

from .platform_observation import PlatformObservation


CONTROL_COMMANDS = ("PING", "PONG")


def _crc16(data: bytes) -> int:
    return binascii.crc_hqx(data, 0xFFFF)


def encode_control(command: str, seq: int) -> bytes:
    command = str(command).upper()
    if command not in CONTROL_COMMANDS:
        raise ValueError(f"unsupported VC1 command: {command}")
    body = f"VC1,{command},{int(seq) & 0xFFFFFFFF}".encode("ascii")
    return body + b"," + f"{_crc16(body):04X}".encode("ascii") + b"\n"


def parse_control(line: bytes | str) -> tuple[str, int]:
    raw = line.encode("ascii") if isinstance(line, str) else bytes(line)
    parts = raw.strip().split(b",")
    if len(parts) != 4 or parts[0] != b"VC1":
        raise ValueError("invalid VC1 field count or prefix")
    body = b",".join(parts[:-1])
    try:
        expected_crc = int(parts[-1], 16)
        command = parts[1].decode("ascii")
        seq = int(parts[2])
    except (UnicodeDecodeError, ValueError) as exc:
        raise ValueError("invalid VC1 field") from exc
    if _crc16(body) != expected_crc:
        raise ValueError("invalid VC1 CRC")
    if command not in CONTROL_COMMANDS or not 0 <= seq <= 0xFFFFFFFF:
        raise ValueError("VC1 field out of range")
    return command, seq


def parse_line(line: bytes | str, received_monotonic: float | None = None) -> PlatformObservation:
    raw = line.encode("ascii") if isinstance(line, str) else bytes(line)
    raw = raw.strip()
    parts = raw.split(b",")
    if len(parts) != 13 or parts[0] != b"VS1":
        raise ValueError("VS1字段数量或版本错误")
    body = b",".join(parts[:-1])
    try:
        expected_crc = int(parts[-1], 16)
    except ValueError as exc:
        raise ValueError("VS1 CRC字段无效") from exc
    if _crc16(body) != expected_crc:
        raise ValueError("VS1 CRC错误")
    try:
        values = [int(part) for part in parts[1:-1]]
    except ValueError as exc:
        raise ValueError("VS1数值字段无效") from exc
    stream_id, seq, capture_ms, found, cx, cy, outer, inner, angle, quality, flags = values
    if stream_id == 0 or found not in (0, 1) or not 0 <= quality <= 100:
        raise ValueError("VS1字段越界")
    if min(outer, inner) < 0 or not 0 <= flags <= 0xFFFF:
        raise ValueError("VS1尺度或flags越界")
    return PlatformObservation(
        stream_id, seq & 0xFFFFFFFF, capture_ms & 0xFFFFFFFF, bool(found),
        cx, cy, outer, inner, angle, quality, flags,
        time.monotonic() if received_monotonic is None else received_monotonic,
    )


@dataclass
class StreamStats:
    accepted: int = 0
    duplicate: int = 0
    rejected: int = 0
    resyncs: int = 0


class ObservationGate:
    """旧帧过滤器；新stream_id需连续三帧确认后才能接管。"""

    def __init__(self, confirm_frames: int = 3) -> None:
        self.confirm_frames = max(2, int(confirm_frames))
        self.active_stream: int | None = None
        self.last_seq: int | None = None
        self._candidate_stream: int | None = None
        self._candidate_last_seq: int | None = None
        self._candidate_count = 0
        self.stats = StreamStats()

    @staticmethod
    def _newer_u32(candidate: int, previous: int) -> bool:
        delta = (candidate - previous) & 0xFFFFFFFF
        return 0 < delta < 0x80000000

    def accept(self, observation: PlatformObservation) -> PlatformObservation | None:
        if self.active_stream is None:
            self.active_stream = observation.stream_id
            self.last_seq = observation.seq
            self.stats.accepted += 1
            return observation
        if observation.stream_id == self.active_stream:
            if self.last_seq is not None and not self._newer_u32(observation.seq, self.last_seq):
                self.stats.duplicate += 1
                return None
            self.last_seq = observation.seq
            self._candidate_stream = None
            self._candidate_count = 0
            self.stats.accepted += 1
            return observation
        if observation.stream_id != self._candidate_stream:
            self._candidate_stream = observation.stream_id
            self._candidate_last_seq = observation.seq
            self._candidate_count = 1
            return None
        if self._candidate_last_seq is None or not self._newer_u32(observation.seq, self._candidate_last_seq):
            self._candidate_count = 0
            self.stats.rejected += 1
            return None
        self._candidate_last_seq = observation.seq
        self._candidate_count += 1
        if self._candidate_count < self.confirm_frames:
            return None
        self.active_stream = observation.stream_id
        self.last_seq = observation.seq
        self._candidate_stream = None
        self._candidate_count = 0
        self.stats.resyncs += 1
        self.stats.accepted += 1
        return observation
