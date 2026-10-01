"""Bounded QR-geometry visual servoing for warehouse slot inspection."""

import os
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class VisionServoConfig:
    """Conservative image-to-flight command mapping.

    The camera is assumed to look at a vertical shelf face.  Consequently
    image X controls lateral flight X and image Y adjusts the altitude target.
    Depth/Y is intentionally not controlled by this monocular loop.
    """

    enabled: bool = False
    timeout_s: float = 3.0
    lost_timeout_s: float = 0.5
    min_lost_frames: int = 5
    center_tolerance_px: float = 45.0
    max_center_jump_px: float = 260.0
    stable_frames: int = 4
    x_kp_cmd_per_px: float = 0.05
    z_kp_m_per_px: float = 0.001
    max_x_command: float = 12.0
    max_lateral_adjust_m: float = 0.25
    max_z_adjust_m: float = 0.20
    min_z_m: float = 0.70
    max_z_m: float = 1.70
    x_sign: int = 1
    z_sign: int = -1
    reverse_x_on_negative_face: bool = True

    def __post_init__(self):
        if self.timeout_s <= 0 or self.lost_timeout_s <= 0 or self.min_lost_frames < 1:
            raise ValueError("视觉伺服超时必须为正数")
        if self.center_tolerance_px <= 0 or self.max_center_jump_px <= 0 or self.stable_frames < 1:
            raise ValueError("视觉伺服居中参数无效")
        if self.x_kp_cmd_per_px <= 0 or self.z_kp_m_per_px <= 0:
            raise ValueError("视觉伺服增益必须为正数")
        if self.max_x_command <= 0 or self.max_lateral_adjust_m <= 0 or self.max_z_adjust_m <= 0:
            raise ValueError("视觉伺服输出上限必须为正数")
        if self.min_z_m >= self.max_z_m:
            raise ValueError("视觉伺服高度上下限无效")
        if self.x_sign not in {-1, 1} or self.z_sign not in {-1, 1}:
            raise ValueError("视觉伺服方向只能是1或-1")

    @classmethod
    def from_env(cls, environ=None):
        env = os.environ if environ is None else environ
        enabled = env.get("DRONE_VISION_SERVO", "0").strip().lower()
        if enabled not in {"0", "1", "false", "true"}:
            raise ValueError("DRONE_VISION_SERVO只能是0/1/false/true")
        return cls(
            enabled=enabled in {"1", "true"},
            timeout_s=float(env.get("DRONE_VISION_SERVO_TIMEOUT_S", "3.0")),
            lost_timeout_s=float(env.get("DRONE_VISION_SERVO_LOST_TIMEOUT_S", "0.5")),
            min_lost_frames=int(env.get("DRONE_VISION_SERVO_MIN_LOST_FRAMES", "5")),
            center_tolerance_px=float(env.get("DRONE_VISION_SERVO_TOLERANCE_PX", "45")),
            max_center_jump_px=float(env.get("DRONE_VISION_SERVO_MAX_CENTER_JUMP_PX", "260")),
            stable_frames=int(env.get("DRONE_VISION_SERVO_STABLE_FRAMES", "4")),
            x_kp_cmd_per_px=float(env.get("DRONE_VISION_SERVO_X_KP", "0.05")),
            z_kp_m_per_px=float(env.get("DRONE_VISION_SERVO_Z_KP", "0.001")),
            max_x_command=float(env.get("DRONE_VISION_SERVO_MAX_X_CMD", "12")),
            max_lateral_adjust_m=float(env.get("DRONE_VISION_SERVO_MAX_LATERAL_ADJUST_M", "0.25")),
            max_z_adjust_m=float(env.get("DRONE_VISION_SERVO_MAX_Z_ADJUST_M", "0.20")),
            min_z_m=float(env.get("DRONE_VISION_SERVO_MIN_Z_M", "0.70")),
            max_z_m=float(env.get("DRONE_VISION_SERVO_MAX_Z_M", "1.70")),
            x_sign=int(env.get("DRONE_VISION_SERVO_X_SIGN", "1")),
            z_sign=int(env.get("DRONE_VISION_SERVO_Z_SIGN", "-1")),
            reverse_x_on_negative_face=env.get(
                "DRONE_VISION_SERVO_REVERSE_X_NEGATIVE_FACE", "1"
            ).strip().lower() in {"1", "true", "yes", "on"},
        )

    def x_direction(self, face) -> int:
        sign = self.x_sign
        if self.reverse_x_on_negative_face and getattr(face, "value", face) in {"B", "D"}:
            sign *= -1
        return sign


@dataclass(frozen=True)
class VisionServoResult:
    success: bool
    z_target_m: float
    frames: int
    stable_frames: int
    error_x_px: Optional[float] = None
    error_y_px: Optional[float] = None
    reason: str = ""


def servo_command(config: VisionServoConfig, error_x_px: float, error_y_px: float,
                  base_z_m: float, current_z_target_m: float, x_direction: int):
    """Calculate one bounded command; this function has no hardware side effects."""
    x_cmd = x_direction * error_x_px * config.x_kp_cmd_per_px
    x_cmd = max(-config.max_x_command, min(config.max_x_command, x_cmd))
    z_target = current_z_target_m + config.z_sign * error_y_px * config.z_kp_m_per_px
    z_target = max(base_z_m - config.max_z_adjust_m, min(base_z_m + config.max_z_adjust_m, z_target))
    z_target = max(config.min_z_m, min(config.max_z_m, z_target))
    return x_cmd, z_target
