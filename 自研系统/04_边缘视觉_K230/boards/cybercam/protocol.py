"""
protocol.py — CyberCAM → Pi UART ASCII 协议

数据帧格式（每行一个检测结果，115200 8-N-1）：

    <dx>,<dy>,<found>\n

字段说明见 README.md。
"""

from __future__ import annotations


# ── 协议常量 ────────────────────────────────────────────────────────── #
BAUDRATE = 115200

FIELD_SEP = b","
LINE_END = b"\n"

# ── 编解码 ──────────────────────────────────────────────────────────── #

def encode(dx: int, dy: int, found: bool) -> bytes:
    """
    Parameters
    ----------
    dx    : 像素偏移 X（右为正），范围 ±1920
    dy    : 像素偏移 Y（下为正），范围 ±1080
    found : 是否检测到目标
    """
    return f"{dx},{dy},{1 if found else 0}\n".encode("ascii")


def parse(line: str) -> tuple[int, int, bool]:
    """
    解析一行 ASCII 数据。返回 (dx, dy, found)。

    Raises
    ------
    ValueError / IndexError : 格式异常
    """
    line = line.strip()
    if not line:
        raise ValueError("empty line")
    parts = line.split(",")
    if len(parts) != 3:
        raise ValueError(f"expected 3 fields, got {len(parts)}: {line}")
    dx = int(parts[0])
    dy = int(parts[1])
    found = parts[2] == "1"
    return dx, dy, found
