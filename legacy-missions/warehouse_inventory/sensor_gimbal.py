"""摄像头/激光共轴舵机云台。货架面显式决定角度，不按识别数量自动翻转。"""

import os
import time
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from Lcode.Logger import logger
from Lcode.warehouse_model import FaceId


class SysfsPWM:
    def __init__(self, chip=0, channel=0, frequency_hz=50, sysfs_root="/sys/class/pwm"):
        self.base = Path(sysfs_root) / f"pwmchip{chip}"
        self.path = self.base / f"pwm{channel}"
        self.channel = int(channel)
        self.period_ns = int(1_000_000_000 / frequency_hz)
        if not self.path.exists():
            (self.base / "export").write_text(str(self.channel), encoding="ascii")
            time.sleep(0.2)
        # On this board, pwm0 starts with period=0 after boot. Writing
        # enable=0 first then raises EINVAL; initialize the period first.
        self._write("period", self.period_ns)
        self._write("enable", 0)

    def _write(self, name, value):
        (self.path / name).write_text(str(int(value)), encoding="ascii")

    def start(self):
        self._write("enable", 1)

    def set_duty_ns(self, duty_ns):
        self._write("duty_cycle", max(0, min(self.period_ns, int(duty_ns))))

    def stop(self):
        self._write("enable", 0)


@dataclass(frozen=True)
class GimbalConfig:
    chip: int = 0
    channel: int = 0
    positive_y_angle_deg: float = 0.0
    negative_y_angle_deg: float = 180.0
    min_pulse_ns: int = 500_000
    max_pulse_ns: int = 2_500_000
    settle_s: float = 0.60

    def __post_init__(self):
        for angle in (self.positive_y_angle_deg, self.negative_y_angle_deg):
            if not 0 <= angle <= 180:
                raise ValueError("云台角度必须在[0,180]度内")
        if not 0 < self.min_pulse_ns < self.max_pulse_ns:
            raise ValueError("云台PWM脉宽范围无效")
        if not 0.1 <= self.settle_s <= 5.0:
            raise ValueError("云台稳定等待必须在[0.1,5]秒内")

    @classmethod
    def from_env(cls, environ: Optional[Mapping[str, str]] = None):
        env = os.environ if environ is None else environ
        return cls(
            chip=int(env.get("DRONE_GIMBAL_PWM_CHIP", "0")),
            channel=int(env.get("DRONE_GIMBAL_PWM_CHANNEL", "0")),
            positive_y_angle_deg=float(env.get("DRONE_GIMBAL_POS_Y_DEG", "0")),
            negative_y_angle_deg=float(env.get("DRONE_GIMBAL_NEG_Y_DEG", "180")),
            min_pulse_ns=int(env.get("DRONE_GIMBAL_MIN_PULSE_NS", "500000")),
            max_pulse_ns=int(env.get("DRONE_GIMBAL_MAX_PULSE_NS", "2500000")),
            settle_s=float(env.get("DRONE_GIMBAL_SETTLE_S", "0.60")),
        )


class SensorGimbal:
    def __init__(self, config: GimbalConfig = None, pwm=None, sleep_fn=time.sleep):
        self.config = config or GimbalConfig.from_env()
        self._pwm = pwm
        self._sleep = sleep_fn
        self.current_angle_deg = None
        self.current_face = None
        self.started = False
    def start(self) -> bool:
        try:
            if self._pwm is None:
                self._pwm = SysfsPWM(self.config.chip, self.config.channel)
            self._pwm.start()
            self.started = True
            return True
        except Exception as exc:
            logger.error(f"云台初始化失败: {exc}")
            self.started = False
            return False

    def _angle_for_face(self, face: FaceId) -> float:
        if face in {FaceId.A, FaceId.C}:
            return self.config.positive_y_angle_deg
        return self.config.negative_y_angle_deg

    def set_face(self, face: FaceId, wait: bool = True) -> bool:
        if not self.started:
            raise RuntimeError("云台尚未启动")
        face = FaceId(face)
        angle = self._angle_for_face(face)
        try:
            pulse = self.config.min_pulse_ns + int(
                angle / 180.0 * (self.config.max_pulse_ns - self.config.min_pulse_ns)
            )
            self._pwm.set_duty_ns(pulse)
            self.current_angle_deg = angle
            self.current_face = face
            logger.info(
                f"云台设置完成: face={face.value}, angle={angle:.1f}deg, "
                f"pulse={pulse}ns"
            )
            if wait:
                self._sleep(self.config.settle_s)
            return True
        except Exception as exc:
            logger.error(f"云台转向{face.value}面失败: {exc}")
            return False

    def close(self):
        if self._pwm is not None:
            try:
                self._pwm.stop()
            except Exception as exc:
                logger.error(f"云台关闭失败: {exc}")
        self.started = False
