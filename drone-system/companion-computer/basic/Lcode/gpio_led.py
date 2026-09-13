"""RGB警示灯GPIO驱动。地瓜派(RDK X5)专用，用`Hobot.GPIO`(不是`wiringpi`/
`RPi.GPIO`)，BCM引脚编号。2026-07-29为避免BCM23（40Pin物理Pin16）
与UART7_RTS冲突，红灯迁移到BCM6（40Pin物理Pin31）；当前接线为
R=BCM6/G=BCM25/B=BCM24。

`Hobot.GPIO`只在板子上能import，本机开发/测试环境(Windows)导入会失败——延迟
导入+try/except，让本机pytest能正常跑(GPIO控制实际降级为空操作)，板子上真实
调用时才会走真实GPIO分支。

跟原始参考代码(`rgb_led(color, duration)`，`time.sleep(duration)`后自动关灯)不同，
这里改成`set_rgb_led(color)`状态型接口，不做阻塞式`time.sleep()`——`Mission_GPT.py`
的`navigate()`每30ms跑一次主循环，函数内阻塞哪怕1秒都会拖慢整个飞控指令节奏，
需要保持点亮状态时调用方自己决定什么时候再调`set_rgb_led('OFF')`关掉。
"""
import threading

from Lcode.Logger import logger

LED_PINS = {'R': 6, 'G': 25, 'B': 24}

_gpio_module = None
_setup_done = False
_lock = threading.Lock()


def _get_gpio():
    """延迟导入`Hobot.GPIO`，本机(无此库)导入失败时返回None，不抛异常。"""
    global _gpio_module
    if _gpio_module is None:
        try:
            import Hobot.GPIO as GPIO
        except ImportError:
            logger.warning("Hobot.GPIO不可用(非板载环境?)，LED控制降级为空操作")
            return None
        _gpio_module = GPIO
    return _gpio_module


def _ensure_setup(gpio):
    global _setup_done
    with _lock:
        if _setup_done:
            return
        gpio.setmode(gpio.BCM)
        for pin in LED_PINS.values():
            gpio.setup(pin, gpio.OUT)
            gpio.output(pin, gpio.LOW)
        _setup_done = True


def set_rgb_led(color: str) -> bool:
    """设置RGB LED颜色并保持点亮，不自动关闭。支持'R'/'G'/'B'/'W'/'OFF'。
    GPIO不可用(本机开发环境)时返回False，不抛异常。"""
    gpio = _get_gpio()
    if gpio is None:
        return False
    _ensure_setup(gpio)

    color_map = {
        'R': {'R': gpio.HIGH, 'G': gpio.LOW, 'B': gpio.LOW},
        'G': {'R': gpio.LOW, 'G': gpio.HIGH, 'B': gpio.LOW},
        'B': {'R': gpio.LOW, 'G': gpio.LOW, 'B': gpio.HIGH},
        'W': {'R': gpio.HIGH, 'G': gpio.HIGH, 'B': gpio.HIGH},
        'OFF': {'R': gpio.LOW, 'G': gpio.LOW, 'B': gpio.LOW},
    }
    key = color.upper()
    if key not in color_map:
        logger.warning(f"set_rgb_led: 不支持的颜色 {color}")
        return False

    states = color_map[key]
    gpio.output(LED_PINS['R'], states['R'])
    gpio.output(LED_PINS['G'], states['G'])
    gpio.output(LED_PINS['B'], states['B'])
    return True
