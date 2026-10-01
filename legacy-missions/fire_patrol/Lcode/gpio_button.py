"""按键GPIO驱动。地瓜派(RDK X5)专用，用`Hobot.GPIO`，BCM引脚编号——参照
`Desktop/GPIO测试/按键测试.ipynb`验证过的接线方案(2026-07-16，BCM17，下降沿触发，
说明按键默认拉高、按下接地)。

`Hobot.GPIO`只在板子上能import，本机开发/测试环境(Windows)导入会失败——
`start()`延迟导入+try/except，本机环境下返回False不抛异常(同`Lcode/gpio_led.py`
的处理方式)。

本模块只是基础GPIO驱动，没有接入任何业务逻辑(比如触发起飞/抛投)——具体怎么用
由调用方决定。
"""
import threading

from Lcode.Logger import logger

BUTTON_PIN_DEFAULT = 17


class GpioButton:
    """轮询式按键状态封装：后台边沿中断计数按下次数，主循环通过
    `was_pressed()`取"自上次调用以来是否按下过"并清零，不需要自己处理
    回调线程同步。"""

    def __init__(self, pin: int = BUTTON_PIN_DEFAULT, bouncetime_ms: int = 200):
        self.pin = pin
        self.bouncetime_ms = bouncetime_ms
        self._lock = threading.Lock()
        self._press_count = 0
        self._gpio = None
        self._available = False

    def start(self) -> bool:
        try:
            import Hobot.GPIO as GPIO
        except ImportError:
            logger.warning("Hobot.GPIO不可用(非板载环境?)，按键检测禁用")
            return False

        self._gpio = GPIO
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN)
        GPIO.add_event_detect(
            self.pin, GPIO.FALLING,
            callback=self._on_press, bouncetime=self.bouncetime_ms,
        )
        self._available = True
        return True

    def _on_press(self, channel):
        with self._lock:
            self._press_count += 1

    def was_pressed(self) -> bool:
        """自上次调用以来是否至少按下过一次，取走计数并清零。"""
        with self._lock:
            pressed = self._press_count > 0
            self._press_count = 0
            return pressed

    def stop(self):
        if self._available and self._gpio is not None:
            self._gpio.remove_event_detect(self.pin)
        self._available = False
