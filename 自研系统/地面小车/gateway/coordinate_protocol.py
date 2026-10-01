"""Payload definitions for car/UAV coordinate transport."""

from __future__ import annotations

from dataclasses import dataclass
import struct


# UAV -> car, defined by car_protocol_delta_for_test.md.
UAV_STATE_STRUCT = struct.Struct("<iih")
UAV_EVENT_STRUCT = struct.Struct("<BI")

# Car -> UAV: x, y, car_pose_age_ms, flags.
CAR_POSITION_STRUCT = struct.Struct("<iiHH")

# Car -> UAV/GROUND: segment, track_s_mm, speed_mm_s, heading_cdeg, flags, vx_mm_s, vy_mm_s.
# 扩展格式 13 字节（任务二 FormationController 前馈需要 vx/vy 世界速度）。
# 基础格式 9 字节 = <BHhhH（不含 vx/vy），扩展格式 13 字节 = <BHhhHhh。
# 注意：speed_mm_s / heading_cdeg / vx / vy 都是有符号 i16，用 h 不用 H。
CAR_STATE_STRUCT = struct.Struct("<BHhhHhh")
CAR_STATE_BASIC_SIZE = 9   # <BHhhH
CAR_STATE_EXT_SIZE = 13    # <BHhhHhh

# Car -> ground: car x/y, UAV x/y/z, original UAV seq/sender time,
# age of the car pose sample and validity flags.
FUSED_POSITION_STRUCT = struct.Struct("<iiiihHIHH")

CAR_POSE_VALID = 1 << 0
UAV_POSE_VALID = 1 << 1
CAR_POSE_FRESH = 1 << 2
SESSION_VALID = 1 << 3


def clamp_i16(value: int) -> int:
    return max(-32768, min(32767, int(value)))


def clamp_u16(value: int) -> int:
    return max(0, min(0xFFFF, int(value)))


@dataclass(frozen=True)
class UavPosition:
    x_mm: int
    y_mm: int
    z_mm: int

    def pack(self) -> bytes:
        return UAV_STATE_STRUCT.pack(self.x_mm, self.y_mm, self.z_mm)

    @classmethod
    def unpack(cls, payload: bytes) -> "UavPosition":
        if len(payload) != UAV_STATE_STRUCT.size:
            raise ValueError(f"UAV_STATE payload must be {UAV_STATE_STRUCT.size} bytes")
        return cls(*UAV_STATE_STRUCT.unpack(payload))


@dataclass(frozen=True)
class CarPosition:
    x_mm: int
    y_mm: int
    pose_age_ms: int
    flags: int

    def pack(self) -> bytes:
        return CAR_POSITION_STRUCT.pack(self.x_mm, self.y_mm, clamp_u16(self.pose_age_ms), self.flags & 0xFFFF)

    @classmethod
    def unpack(cls, payload: bytes) -> "CarPosition":
        if len(payload) != CAR_POSITION_STRUCT.size:
            raise ValueError(f"CAR_POSITION payload must be {CAR_POSITION_STRUCT.size} bytes")
        return cls(*CAR_POSITION_STRUCT.unpack(payload))


@dataclass(frozen=True)
class CarState:
    """CAR_STATE (0x10) 载荷。

    扩展格式 13 字节（含 vx/vy，任务二 FormationController 前馈需要）：
      segment:u8, track_s_mm:u16, speed_mm_s:i16, heading_cdeg:i16,
      flags:u16, vx_mm_s:i16, vy_mm_s:i16

    基础格式 9 字节（不含 vx/vy，任务一足够）：
      segment:u8, track_s_mm:u16, speed_mm_s:i16, heading_cdeg:i16, flags:u16

    字段含义：
      segment       当前赛道区段编号
      track_s_mm    当前赛道行程（mm）
      speed_mm_s    小车当前速度（mm/s，有符号，正=前进）
      heading_cdeg  小车航向（百分之一度，0=+X 方向）
      flags         状态位
      vx_mm_s       小车世界坐标 X 方向速度（mm/s，有符号）
      vy_mm_s       小车世界坐标 Y 方向速度（mm/s，有符号）
    """
    segment: int
    track_s_mm: int
    speed_mm_s: int
    heading_cdeg: int
    flags: int
    vx_mm_s: int = 0
    vy_mm_s: int = 0

    def pack(self) -> bytes:
        """打包为 13 字节扩展格式。"""
        return CAR_STATE_STRUCT.pack(
            self.segment & 0xFF,
            clamp_u16(self.track_s_mm),
            clamp_i16(self.speed_mm_s),
            clamp_i16(self.heading_cdeg),
            self.flags & 0xFFFF,
            clamp_i16(self.vx_mm_s),
            clamp_i16(self.vy_mm_s),
        )

    @classmethod
    def unpack(cls, payload: bytes) -> "CarState":
        """解包，兼容 9 字节基础格式和 13 字节扩展格式。"""
        if len(payload) == CAR_STATE_EXT_SIZE:
            return cls(*CAR_STATE_STRUCT.unpack(payload))
        if len(payload) == CAR_STATE_BASIC_SIZE:
            # 基础格式 <BHhhH，补 vx/vy=0
            basic = struct.Struct("<BHhhH")
            seg, track, speed, heading, flags = basic.unpack(payload)
            return cls(seg, track, speed, heading, flags, 0, 0)
        raise ValueError(
            f"CAR_STATE payload must be {CAR_STATE_BASIC_SIZE} or {CAR_STATE_EXT_SIZE} bytes, got {len(payload)}"
        )


@dataclass(frozen=True)
class FusedPosition:
    car_x_mm: int
    car_y_mm: int
    uav_x_mm: int
    uav_y_mm: int
    uav_z_mm: int
    uav_seq: int
    uav_sender_ms: int
    car_pose_age_ms: int
    flags: int

    def pack(self) -> bytes:
        return FUSED_POSITION_STRUCT.pack(
            self.car_x_mm,
            self.car_y_mm,
            self.uav_x_mm,
            self.uav_y_mm,
            self.uav_z_mm,
            self.uav_seq & 0xFFFF,
            self.uav_sender_ms & 0xFFFFFFFF,
            clamp_u16(self.car_pose_age_ms),
            self.flags & 0xFFFF,
        )

    @classmethod
    def unpack(cls, payload: bytes) -> "FusedPosition":
        if len(payload) != FUSED_POSITION_STRUCT.size:
            raise ValueError(f"FUSED_POSITION payload must be {FUSED_POSITION_STRUCT.size} bytes")
        return cls(*FUSED_POSITION_STRUCT.unpack(payload))
