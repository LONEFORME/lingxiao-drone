import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from Lcode.gpio_button import GpioButton, BUTTON_PIN_DEFAULT


def _gpio_available():
    try:
        import Hobot.GPIO  # noqa: F401
    except ImportError:
        return False
    return True


class TestGpioButtonStart:
    def test_start_matches_gpio_availability_on_this_machine(self):
        """本机(Windows)没有Hobot.GPIO，start()应该优雅降级返回False；板载环境
        有Hobot.GPIO时走真实GPIO分支返回True。结果应该跟这台机器上Hobot.GPIO
        是否真的可导入一致，不能反过来，也不应该抛异常。"""
        btn = GpioButton()
        try:
            result = btn.start()
            assert result is _gpio_available()
        finally:
            btn.stop()

    def test_was_pressed_false_before_start(self):
        btn = GpioButton()
        assert btn.was_pressed() is False

    def test_default_pin_matches_reference_wiring(self):
        """默认引脚应该跟Desktop/GPIO测试/按键测试.ipynb验证过的接线(BCM17)一致。"""
        assert BUTTON_PIN_DEFAULT == 17

    def test_stop_before_start_does_not_raise(self):
        btn = GpioButton()
        btn.stop()  # 不应该抛异常
