"""BCM19激光安全驱动：启动/异常/关闭默认LOW，0.5秒脉冲在后台线程运行。"""

import os
import threading
import time
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Optional

from Lcode.Logger import logger


@dataclass(frozen=True)
class LaserConfig:
    enabled: bool = True
    pin: int = 19
    duration_s: float = 0.50
    duty_ratio: float = 0.10
    pwm_period_s: float = 0.02

    def __post_init__(self):
        if not 0.1 <= self.duration_s <= 0.7:
            raise ValueError("激光单次点亮必须在[0.1,0.7]秒内")
        if not 0 < self.duty_ratio <= 1.0:
            raise ValueError("激光PWM占空比必须在(0,1]内")
        if not 0.005 <= self.pwm_period_s <= 0.1:
            raise ValueError("激光PWM周期必须在[0.005,0.1]秒内")

    @classmethod
    def from_env(cls, environ: Optional[Mapping[str, str]] = None):
        env = os.environ if environ is None else environ
        enabled = env.get("DRONE_LASER_ENABLED", "1").strip().lower()
        if enabled not in {"0", "1", "false", "true"}:
            raise ValueError("DRONE_LASER_ENABLED只能是0/1/false/true")
        return cls(
            enabled=enabled in {"1", "true"},
            pin=int(env.get("DRONE_LASER_PIN", "19")),
            duration_s=float(env.get("DRONE_LASER_DURATION_S", "0.50")),
            duty_ratio=float(env.get("DRONE_LASER_DUTY", "0.10")),
            pwm_period_s=float(env.get("DRONE_LASER_PWM_PERIOD_S", "0.02")),
        )


class LaserPointer:
    def __init__(self, config: LaserConfig = None, gpio=None):
        self.config = config or LaserConfig.from_env()
        self._gpio = gpio
        self._thread = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self.started = False
        self.active = False

    def start(self) -> bool:
        if not self.config.enabled:
            self.started = True
            return True
        try:
            if self._gpio is None:
                import Hobot.GPIO as GPIO

                self._gpio = GPIO
            self._gpio.setmode(self._gpio.BCM)
            self._gpio.setup(self.config.pin, self._gpio.OUT)
            self._gpio.output(self.config.pin, self._gpio.LOW)
            self.started = True
            return True
        except Exception as exc:
            logger.error(f"激光GPIO初始化失败: {exc}")
            self.started = False
            return False

    def _write_low(self):
        if self.config.enabled and self._gpio is not None:
            self._gpio.output(self.config.pin, self._gpio.LOW)

    def pulse_async(self, duration_s: Optional[float] = None) -> bool:
        if not self.started:
            raise RuntimeError("激光驱动尚未启动")
        if not self.config.enabled:
            return True
        duration = self.config.duration_s if duration_s is None else float(duration_s)
        if not 0.1 <= duration <= 0.7:
            raise ValueError("激光单次点亮必须在[0.1,0.7]秒内")
        with self._lock:
            if self.active:
                return False
            self.active = True
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._pulse_worker,
                args=(duration,),
                daemon=True,
                name="laser-pulse",
            )
            self._thread.start()
        return True

    def _pulse_worker(self, duration):
        on_time = self.config.pwm_period_s * self.config.duty_ratio
        off_time = self.config.pwm_period_s - on_time
        deadline = time.monotonic() + duration
        # 激光照射时同步点亮绿灯，便于地面观察确认扫码触发
        try:
            from Lcode.gpio_led import set_rgb_led
            set_rgb_led('G')
        except Exception:
            pass
        try:
            while not self._stop_event.is_set() and time.monotonic() < deadline:
                self._gpio.output(self.config.pin, self._gpio.HIGH)
                if self._stop_event.wait(on_time):
                    break
                self._gpio.output(self.config.pin, self._gpio.LOW)
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._stop_event.wait(min(off_time, remaining))
        except Exception as exc:
            logger.error(f"激光脉冲异常，已强制关闭: {exc}")
        finally:
            try:
                self._write_low()
            finally:
                with self._lock:
                    self.active = False
            try:
                from Lcode.gpio_led import set_rgb_led
                set_rgb_led('OFF')
            except Exception:
                pass

    def off(self):
        self._stop_event.set()
        try:
            self._write_low()
        except Exception as exc:
            logger.error(f"激光强制关闭失败: {exc}")

    def wait(self, timeout=1.0) -> bool:
        thread = self._thread
        if thread and thread.is_alive():
            thread.join(timeout=timeout)
        return not self.active

    def close(self):
        self.off()
        self.wait(timeout=1.0)
        if self.config.enabled and self._gpio is not None:
            try:
                self._gpio.cleanup()
            except Exception as exc:
                logger.error(f"激光GPIO清理失败: {exc}")
        self.started = False
