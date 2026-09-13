"""Cyber Camera到Pi的VS1 ASCII协议。"""

from __future__ import annotations

import binascii


BAUDRATE = 115200
FIELD_COUNT = 13
CONTROL_PREFIX = b"VC1"
CONTROL_COMMANDS = ("PING", "PONG")


def crc16(data: bytes) -> int:
    return binascii.crc_hqx(bytes(data), 0xFFFF)


def encode_control(command: str, seq: int) -> bytes:
    command = str(command).upper()
    if command not in CONTROL_COMMANDS:
        raise ValueError(f"unsupported VC1 command: {command}")
    body = f"VC1,{command},{int(seq) & 0xFFFFFFFF}".encode("ascii")
    return body + b"," + f"{crc16(body):04X}".encode("ascii") + b"\n"


def parse_control(line: bytes | str) -> tuple[str, int]:
    raw = line.encode("ascii") if isinstance(line, str) else bytes(line)
    parts = raw.strip().split(b",")
    if len(parts) != 4 or parts[0] != CONTROL_PREFIX:
        raise ValueError("invalid VC1 field count or prefix")
    body = b",".join(parts[:-1])
    try:
        expected_crc = int(parts[-1], 16)
        command = parts[1].decode("ascii")
        seq = int(parts[2])
    except (UnicodeDecodeError, ValueError) as exc:
        raise ValueError("invalid VC1 field") from exc
    if crc16(body) != expected_crc:
        raise ValueError("invalid VC1 CRC")
    if command not in CONTROL_COMMANDS or not 0 <= seq <= 0xFFFFFFFF:
        raise ValueError("VC1 field out of range")
    return command, seq


def encode(
    stream_id: int,
    seq: int,
    capture_ms: int,
    found: bool,
    cx: int,
    cy: int,
    outer_px: int,
    inner_px: int,
    angle_cdeg: int,
    quality: int,
    flags: int,
) -> bytes:
    values = (
        "VS1",
        str(int(stream_id) & 0xFFFF),
        str(int(seq) & 0xFFFFFFFF),
        str(int(capture_ms) & 0xFFFFFFFF),
        "1" if found else "0",
        str(int(cx)),
        str(int(cy)),
        str(max(0, int(outer_px))),
        str(max(0, int(inner_px))),
        str(int(angle_cdeg)),
        str(max(0, min(100, int(quality)))),
        str(int(flags) & 0xFFFF),
    )
    body = ",".join(values).encode("ascii")
    return body + b"," + f"{crc16(body):04X}".encode("ascii") + b"\n"
