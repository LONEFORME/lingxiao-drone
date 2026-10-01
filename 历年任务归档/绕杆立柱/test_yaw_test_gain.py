"""问题16排查：yaw修正回路低增益递进式复测开关(YAW_TEST_KP)。

默认(YAW_TEST_KP=0)必须保持test_yaw_unit_fix.py验证过的安全回退行为不变——
喂弧度，正常飞行范围内的yaw误差恒产生0修正指令。只有显式设置YAW_TEST_KP>0
才切换成喂角度+这个低增益，用于问题16的真机复测，绝不能悄悄变成默认行为。

2026-07-12曾短暂把默认值改为0.45再改为0.4正式启用，但事后对比历史"完全不
修正"基线(凌霄IMU纯姿态自稳，yaw峰值6.12°会自己回归)发现，今天"开启修正"的
样本(峰值6.13°~10.38°)并不比不修正基线更好，缺乏证据支持修正有效，而下行
风险(Kp=0.5发散、Kp=0.45方差大)是明确的——已改回默认禁用。

运行：
    cd drone_control/circle_pole && python -m pytest test_yaw_test_gain.py -v
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import Mission_GPT as mg
from Mission_GPT import mission


class FakeRealsenseYaw:
    def __init__(self, confidence=3, vel=(0.0, 0.0, 0.0)):
        self._confidence = confidence
        self._vel = vel

    def get_tracking_confidence(self):
        return self._confidence

    def get_velocity(self):
        return list(self._vel)


def _make_mission():
    re_fc = [0] * 14
    se_fc = [0] * 11
    m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None, radar_obj=None)
    m.t265_ok = True
    m.realsense = FakeRealsenseYaw()
    return m


class TestYawTestGainDisabledByDefault:
    def test_default_kp_is_zero(self):
        """模块级默认值必须是0(关闭)，不能被意外改成非零默认值。"""
        assert mg.YAW_TEST_KP == 0

    def test_mission_not_flagged_test_enabled_by_default(self):
        m = _make_mission()
        assert m._yaw_test_enabled is False


class TestYawTestGainOptIn:
    def test_enabled_with_low_gain_produces_nonzero_correction(self, monkeypatch):
        """显式启用(如Kp=0.3)时，今天真机实测过的yaw误差(6.12度)应该能产生
        非零修正指令(不再被int()截断成0)——这是"重新真正生效"的直接验证。"""
        monkeypatch.setattr(mg, "YAW_TEST_KP", 0.3)
        m = _make_mission()
        assert m._yaw_test_enabled is True

        target = m.targets[0]
        sent = []
        m.set_speed = lambda *a, **k: sent.append(a)

        yaw_rad = math.radians(-6.12)
        m.navigate(list(target), yaw_rad)

        assert len(sent) == 1
        vyaw_sent = sent[0][2]
        assert vyaw_sent != 0

    def test_disabled_still_produces_zero_correction_when_monkeypatched_back(self, monkeypatch):
        """确认开关本身双向生效：显式设回0，行为应该跟未设置时完全一致(仍是0)。"""
        monkeypatch.setattr(mg, "YAW_TEST_KP", 0)
        m = _make_mission()
        assert m._yaw_test_enabled is False

        target = m.targets[0]
        sent = []
        m.set_speed = lambda *a, **k: sent.append(a)

        yaw_rad = math.radians(-6.12)
        m.navigate(list(target), yaw_rad)

        assert sent[0][2] == 0
