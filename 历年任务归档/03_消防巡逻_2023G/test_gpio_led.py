import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Lcode.gpio_led import set_rgb_led, _get_gpio


def _gpio_available():
    return _get_gpio() is not None


class TestSetRgbLed:
    def test_does_not_raise_and_returns_bool(self):
        """两边环境都要能跑：本机(Windows)没有Hobot.GPIO，降级返回False；
        板载环境有Hobot.GPIO时走真实GPIO分支返回True(2026-07-16板载实测确认
        真的点亮了LED，之后手动关闭)。不管哪边，都不应该抛异常。"""
        result = set_rgb_led('R')
        assert isinstance(result, bool)
        set_rgb_led('OFF')  # 不留灯亮着，两边环境都执行(本机是空操作)

    def test_matches_gpio_availability_on_this_machine(self):
        """结果应该跟这台机器上Hobot.GPIO是否真的可导入一致，不能反过来。"""
        expected = _gpio_available()
        result = set_rgb_led('R')
        set_rgb_led('OFF')
        assert result is expected

    def test_unsupported_color_returns_false_when_gpio_available(self):
        if not _gpio_available():
            pytest.skip("需要真实Hobot.GPIO才能测到颜色校验分支，本机跳过")
        assert set_rgb_led('PURPLE') is False
