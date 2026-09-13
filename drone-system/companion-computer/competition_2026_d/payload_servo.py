"""将D题真实PWM舵机适配为一次性PayloadActuator。"""

from __future__ import annotations

import time
from typing import Callable

from .Lcode.gpio_servo import (
    SERVO_ANGLE_CLOSED,
    SERVO_ANGLE_OPEN,
    set_servo_angle,
)
from .payload_actuator import PayloadActuator, PayloadActuatorConfig


class ServoPayloadHardware:
    """释放后等待物体脱离，再自动回到锁定角度。

    机构没有独立位置反馈，因此这里的True只表示PWM写入和延时复位均成功，
    不代表传感器独立确认物体已经落下。
    """

    def __init__(
        self,
        *,
        release_hold_s: float = 1.0,
        clock: Callable[[], float] = time.monotonic,
        angle_writer: Callable[[float], bool] = set_servo_angle,
    ) -> None:
        self.release_hold_s = max(0.1, float(release_hold_s))
        self.clock = clock
        self.angle_writer = angle_writer
        self.ready = False
        self._release_started: float | None = None
        self._release_write_ok = False
        self._closed_after_release = False

    def write_command(self, command: float) -> None:
        if float(command) >= 0.5:
            self._release_started = self.clock()
            self._closed_after_release = False
            self._release_write_ok = bool(
                self.angle_writer(SERVO_ANGLE_OPEN)
            )
        else:
            self.ready = bool(self.angle_writer(SERVO_ANGLE_CLOSED))

    def released_feedback(self) -> bool | None:
        if self._release_started is None:
            return False
        if not self._release_write_ok:
            return None
        if self.clock() - self._release_started < self.release_hold_s:
            return False
        if not self._closed_after_release:
            self._closed_after_release = bool(
                self.angle_writer(SERVO_ANGLE_CLOSED)
            )
            self.ready = self._closed_after_release
        return True if self._closed_after_release else None


def build_payload_actuator(
    *,
    release_hold_s: float = 1.0,
    clock: Callable[[], float] = time.monotonic,
    angle_writer: Callable[[float], bool] = set_servo_angle,
) -> tuple[PayloadActuator, ServoPayloadHardware]:
    hardware = ServoPayloadHardware(
        release_hold_s=release_hold_s,
        clock=clock,
        angle_writer=angle_writer,
    )
    actuator = PayloadActuator(
        hardware.write_command,
        hardware.released_feedback,
        config=PayloadActuatorConfig(
            locked_command=0.0,
            release_command=1.0,
            feedback_timeout_s=release_hold_s + 0.8,
        ),
        clock=clock,
    )
    return actuator, hardware
