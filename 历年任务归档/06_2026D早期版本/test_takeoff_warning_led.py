"""起飞前警示灯(_blink_warning_led)的单元测试。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/basic && python -m pytest test_takeoff_warning_led.py -v
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from Mission_GPT import mission


def _make_mission(tmp_path):
    re_fc = [0] * 14
    se_fc = [0] * 11
    m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None)
    m._log_file = None
    return m


class TestTakeoffWarningLed:
    """起飞前红灯常亮TAKEOFF_WARN_LED_DURATION_S秒提醒周围人员，见_blink_warning_led()。"""

    def test_lights_red_then_off(self, tmp_path, monkeypatch):
        calls = []
        monkeypatch.setattr(
            "Lcode.gpio_led.set_rgb_led",
            lambda color: calls.append(color) or True,
        )
        monkeypatch.setattr("Mission_GPT.time.sleep", lambda s: None)  # 跳过真实sleep，测试不用等5秒

        m = _make_mission(tmp_path)
        assert m._blink_warning_led() is True

        assert calls == ['R', 'OFF']

    def test_gpio_unavailable_does_not_raise(self, tmp_path, monkeypatch):
        """gpio_led不可用时返回失败，让一键起飞门禁阻断起飞。"""
        monkeypatch.setattr("Lcode.gpio_led.set_rgb_led", lambda color: False)
        monkeypatch.setattr("Mission_GPT.time.sleep", lambda s: None)
        m = _make_mission(tmp_path)
        assert m._blink_warning_led() is False
