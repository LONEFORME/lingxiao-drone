"""D题抛投舵机驱动。

硬件接线沿用火源题已实测配置：RDK X5 ``pwmchip0/pwm0``（物理 Pin32），
50 Hz。当前机构的真实角度语义为 180°锁定、0°释放。
"""

from __future__ import annotations

import os
import logging
import threading
import time

logger = logging.getLogger(__name__)

PWM_CHIP = 0
PWM_CHANNEL = 0
PWM_FREQUENCY_HZ = 50
SERVO_ANGLE_CLOSED = 180
SERVO_ANGLE_OPEN = 0

_PWM_BASE = f"/sys/class/pwm/pwmchip{PWM_CHIP}"
_PWM_PATH = f"{_PWM_BASE}/pwm{PWM_CHANNEL}"
_PERIOD_NS = int(1_000_000_000 / PWM_FREQUENCY_HZ)
_setup_done = False
_lock = threading.Lock()


def _write(name: str, value) -> bool:
    try:
        with open(f"{_PWM_PATH}/{name}", "w") as handle:
            handle.write(str(value))
        return True
    except OSError as exc:
        logger.warning(f"gpio_servo: 写{name}失败({exc})")
        return False


def _ensure_setup() -> bool:
    global _setup_done
    with _lock:
        if _setup_done:
            return True
        if not os.path.exists(_PWM_PATH):
            try:
                with open(f"{_PWM_BASE}/export", "w") as handle:
                    handle.write(str(PWM_CHANNEL))
                time.sleep(0.2)
            except OSError as exc:
                logger.warning(f"gpio_servo: export失败({exc})")
                return False
        # 新PWM通道的period为0时，必须先写period再写enable=0。
        if not _write("period", _PERIOD_NS):
            return False
        if not _write("enable", 0):
            return False
        _setup_done = True
        return True


def _angle_to_duty_ns(angle: float) -> int:
    angle = max(0.0, min(180.0, float(angle)))
    return 500_000 + int(angle / 180.0 * 2_000_000)


def set_servo_angle(angle: float) -> bool:
    """设置并保持舵机角度；硬件不可用时返回False，不抛异常。"""
    if not _ensure_setup():
        return False
    if not _write("duty_cycle", _angle_to_duty_ns(angle)):
        return False
    return _write("enable", 1)
