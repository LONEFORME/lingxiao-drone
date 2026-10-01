"""DCP v1 decoder for the 2026 EDC D-topic ground station.

The ground station is deliberately receive-only: this module only decodes,
validates and classifies telemetry frames; it never constructs control frames.
"""

from __future__ import annotations

from dataclasses import dataclass
import struct
from typing import Iterable


MAGIC = 0xAA
TAIL = 0xFF
VERSION = 0x01
MAX_PAYLOAD = 256
HEADER_FORMAT = "<BBBBBIHIH"  # version ... payload_len, excluding magic
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
MIN_FRAME_SIZE = 1 + HEADER_SIZE + 2 + 1

UAV = 0x01
CAR = 0x02
GROUND = 0x03

HEARTBEAT = 0x01
UAV_READY = 0x02
CAR_START = 0x03
ACK = 0x04
CAR_STATE = 0x10
UAV_STATE = 0x11
FUSED_POSITION = 0x12
CAR_POSITION = 0x13
UAV_EVENT = 0x20
# Compatibility alias for logs produced from the older protocol draft.
DROP_RELEASED = UAV_EVENT
TOUCHDOWN_CONFIRMED = 0x21
RETAKEOFF_STARTED = 0x22
MISSION_COMPLETE = 0x23
FAULT_EVENT = 0x30

EVENT_TYPES = {UAV_EVENT, TOUCHDOWN_CONFIRMED, RETAKEOFF_STARTED, MISSION_COMPLETE, FAULT_EVENT}

# DCP v1 section 6: the only legal values of UAV_STATE.phase.
UAV_PHASE_NAMES = {
    0: "BOOT（启动）", 1: "WAIT_T265（等待T265）", 2: "READY（就绪）", 3: "TAKEOFF（起飞）",
    4: "HOLD_3S（悬停3秒）", 5: "INTERCEPT（拦截）", 6: "FORMATION_FOLLOW（伴飞）",
    7: "DROP（抛投）", 8: "DESCEND（下降）", 9: "TOUCHDOWN（触地）",
    10: "DECK_RIDE（平台停留）", 11: "RETAKEOFF（再次起飞）", 12: "RETURN_H（返回H点）",
    13: "LAND_H（降落H点）", 14: "COMPLETE（完成）", 15: "FAULT（故障）",
    18: "SEARCH_TARGET（搜索目标）",
}
VALID_UAV_PHASES = frozenset(UAV_PHASE_NAMES)


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    """CRC-CCITT-FALSE, polynomial 0x1021, MSB first."""
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

    @property
    def is_event(self) -> bool:
        return bool(self.flags & 0x04) or self.message_type in EVENT_TYPES


class DcpStreamParser:
    """Non-blocking length-driven stream parser with byte-wise re-synchronization."""

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
                # Retain nothing: a following serial read will contain the next magic.
                self._buffer.clear()
                break
            if magic_index:
                del self._buffer[:magic_index]
            if len(self._buffer) < MIN_FRAME_SIZE:
                break

            version, msg_type, flags, source, dest, session_id, seq, sender_ms, payload_len = struct.unpack_from(
                HEADER_FORMAT, self._buffer, 1
            )
            if version != VERSION or payload_len > MAX_PAYLOAD:
                self.bad_format += 1
                del self._buffer[0]
                continue

            frame_size = 1 + HEADER_SIZE + payload_len + 2 + 1
            if len(self._buffer) < frame_size:
                break
            if self._buffer[frame_size - 1] != TAIL:
                self.bad_format += 1
                del self._buffer[0]
                continue

            payload_end = 1 + HEADER_SIZE + payload_len
            crc_received = struct.unpack_from("<H", self._buffer, payload_end)[0]
            crc_actual = crc16_ccitt(bytes(self._buffer[1:payload_end]))
            if crc_received != crc_actual:
                self.bad_crc += 1
                del self._buffer[0]
                continue

            raw = bytes(self._buffer[:frame_size])
            frames.append(DcpFrame(msg_type, flags, source, dest, session_id, seq, sender_ms,
                                   bytes(self._buffer[1 + HEADER_SIZE:payload_end]), raw))
            del self._buffer[:frame_size]
        return frames


def unpack_payload(frame: DcpFrame) -> dict[str, int]:
    """Decode every DCP v1 payload used by the display; unknown payloads stay raw."""
    p = frame.payload
    try:
        if frame.message_type == HEARTBEAT and len(p) == 3:
            state, fault_bits = struct.unpack("<BH", p)
            return {"state": state, "fault_bits": fault_bits}
        if frame.message_type == UAV_READY and len(p) == 7:
            task_mask, ready_bits, config_hash = struct.unpack("<BHI", p)
            return {"task_mask": task_mask, "ready_bits": ready_bits, "config_hash": config_hash}
        if frame.message_type == CAR_START and len(p) == 5:
            task_mode, car_config_hash = struct.unpack("<BI", p)
            return {"task_mode": task_mode, "car_config_hash": car_config_hash}
        if frame.message_type == ACK and len(p) == 4:
            acked_type, acked_seq, result = struct.unpack("<BHB", p)
            return {"acked_type": acked_type, "acked_seq": acked_seq, "result": result}
        if frame.message_type == CAR_STATE and len(p) == 9:
            segment, track_s_mm, speed_mm_s, heading_cdeg, flags = struct.unpack("<BHh hH".replace(" ", ""), p)
            return {"segment": segment, "track_s_mm": track_s_mm, "speed_mm_s": speed_mm_s,
                    "heading_cdeg": heading_cdeg, "flags": flags}
        # 1 + 4 + 4 + 2 + 2 + 2 + 1 + 2 = 18 bytes.
        if frame.message_type == UAV_STATE and len(p) == 18:
            phase, x_mm, y_mm, z_mm, vx_mm_s, vy_mm_s, quality, flags = struct.unpack("<Bii hhhBH".replace(" ", ""), p)
            return {"phase": phase, "x_mm": x_mm, "y_mm": y_mm, "z_mm": z_mm,
                    "vx_mm_s": vx_mm_s, "vy_mm_s": vy_mm_s, "vision_quality": quality, "flags": flags}
        # Task-one delta input. Coordinates are relative to the UAV's H-point
        # T265 origin; normally the car converts them before emitting 0x12.
        if frame.message_type == UAV_STATE and len(p) == 10:
            h_x_mm, h_y_mm, z_mm = struct.unpack("<iih", p)
            return {"h_x_mm": h_x_mm, "h_y_mm": h_y_mm, "z_mm": z_mm}
        # DCP-CFP v1 car gateway output:
        # car x/y, UAV x/y/z, original UAV seq/time, car-pose age, validity flags.
        if frame.message_type == FUSED_POSITION and len(p) == 28:
            values = struct.unpack("<iiiihHIHH", p)
            keys = (
                "car_x_mm", "car_y_mm", "uav_x_mm", "uav_y_mm", "uav_z_mm",
                "uav_seq", "uav_sender_ms", "car_pose_age_ms", "position_flags",
            )
            return dict(zip(keys, values))
        if frame.message_type == CAR_POSITION and len(p) == 12:
            car_x_mm, car_y_mm, car_pose_age_ms, position_flags = struct.unpack("<iiHH", p)
            return {
                "car_x_mm": car_x_mm, "car_y_mm": car_y_mm,
                "car_pose_age_ms": car_pose_age_ms, "position_flags": position_flags,
            }
        # The task-one delta replaces the older DROP_RELEASED(0x20) payload
        # with one unified phase event. New traffic always takes precedence.
        if frame.message_type == UAV_EVENT and len(p) == 5:
            phase, elapsed_ms = struct.unpack("<BI", p)
            if phase in VALID_UAV_PHASES:
                return {"phase": phase, "elapsed_ms": elapsed_ms}
            # Best-effort replay compatibility for old logs/simulators. The
            # two historical 0x20 layouts have the same length, so a legal
            # new phase byte is always interpreted as the current protocol.
            legacy_elapsed_ms, quality = struct.unpack("<IB", p)
            return {"legacy_drop": 1, "elapsed_ms": legacy_elapsed_ms, "quality": quality}
        if frame.message_type == TOUCHDOWN_CONFIRMED and len(p) == 5:
            elapsed_ms, confidence = struct.unpack("<IB", p)
            return {"elapsed_ms": elapsed_ms, "confidence": confidence}
        if frame.message_type == RETAKEOFF_STARTED and len(p) == 4:
            return {"elapsed_ms": struct.unpack("<I", p)[0]}
        if frame.message_type == MISSION_COMPLETE and len(p) == 5:
            result, elapsed_ms = struct.unpack("<BI", p)
            return {"result": result, "elapsed_ms": elapsed_ms}
        if frame.message_type == FAULT_EVENT and len(p) == 5:
            fault_code, severity, detail = struct.unpack("<HBH", p)
            return {"fault_code": fault_code, "severity": severity, "detail": detail}
    except struct.error:
        pass
    return {"payload_hex": p.hex(" ")}


def source_name(source: int) -> str:
    return {UAV: "无人机", CAR: "小车", GROUND: "地面站"}.get(source, f"设备0x{source:02X}")


def message_name(message_type: int) -> str:
    names = {
        HEARTBEAT: "心跳", UAV_READY: "无人机就绪", CAR_START: "小车启动", ACK: "确认",
        CAR_STATE: "小车状态", UAV_STATE: "无人机状态", FUSED_POSITION: "陆空融合坐标",
        CAR_POSITION: "小车坐标", UAV_EVENT: "无人机阶段事件",
        TOUCHDOWN_CONFIRMED: "触地确认", RETAKEOFF_STARTED: "二次起飞", MISSION_COMPLETE: "任务完成",
        FAULT_EVENT: "故障事件",
    }
    return names.get(message_type, f"未知0x{message_type:02X}")


def build_frame(message_type: int, payload: bytes, *, source: int = UAV, destination: int = GROUND,
                session_id: int = 1, seq: int = 1, sender_ms: int = 0, flags: int = 0) -> bytes:
    """Test helper; the application itself never calls this to transmit data."""
    header = struct.pack(HEADER_FORMAT, VERSION, message_type, flags, source, destination,
                         session_id, seq, sender_ms, len(payload))
    return bytes([MAGIC]) + header + payload + struct.pack("<H", crc16_ccitt(header + payload)) + bytes([TAIL])
