"""yaw 修正回路弧度/角度单位不匹配 — 修复已回退，本文件测试"回退后的安全状态"。

背景：`yaw_pid` 用的增益(Kp=1.5等)是按"角度"量级设计的，但 `navigate()`/
`takeoff()` 里喂给它的 `yaw` 参数是T265原始朝向，单位是弧度。几度的误差
换算成弧度只有0.05-0.1量级，PID输出乘VEL_SCALE后还是远小于1，最后
`int(...)`截断成0——导致这条回路在正常飞行范围内(误差几度)完全不发指令。

2026-07-09用原始Kp=1.5闭环触发过近90°失控事故，长期回退成喂弧度的"恒输出
≈0"安全状态。2026-07-12递进式真机复测(单独会话+小步长+人工全程待命)确认了
稳定性边界在[0.45,0.5]之间(Kp=0.3/0.4/0.45均收敛，Kp=0.5确认无界发散)，
一度把Kp=0.4设为默认值正式启用——但事后对比历史"完全不修正"基线(峰值6.12°，
会自己回归)发现，"开启修正"的样本(峰值6.13°~10.38°)并不比不修正基线更好，
缺乏证据支持修正确实有效，而下行风险是明确的，**已改回默认禁用**。

**这个文件的用途**：作为默认禁用状态的回归守卫——如果以后有人不小心把
`navigate()`/`takeoff()` 里的 `self.yaw_pid.get_pid(yaw)` 改成默认启用，
这里的测试会失败，提醒这个改动需要先有能证明"修正比不修正好"的对照证据。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/basic_radar && python -m pytest test_yaw_unit_fix.py -v
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

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


class TestYawUnitFixReverted:
    def test_realistic_yaw_error_still_produces_zero_correction(self):
        """默认禁用状态下：今天真机实测过的yaw误差(6.12度)，换算成弧度后
        PID输出仍会被int()截断成0——这是当前刻意维持的安全(但也意味着无效)
        状态，不是bug。如果这个断言开始失败，说明yaw_pid又被改成默认接收
        角度了，必须先有对照证据证明修正确实比不修正好，才能这么做。"""
        m = _make_mission()
        target = m.targets[0]  # 默认航点，跟pos一致，只有yaw有误差
        sent = []
        m.set_speed = lambda *a, **k: sent.append(a)

        yaw_rad = math.radians(-6.12)
        m.navigate(list(target), yaw_rad)

        assert len(sent) == 1
        vyaw_sent = sent[0][2]
        assert vyaw_sent == 0

    def test_small_yaw_error_still_zero(self):
        """误差很小(0.5度)时输出仍然是0，跟死区行为一致。"""
        m = _make_mission()
        target = m.targets[0]
        sent = []
        m.set_speed = lambda *a, **k: sent.append(a)

        yaw_rad = math.radians(0.05)
        m.navigate(list(target), yaw_rad)

        assert len(sent) == 1
        assert sent[0][2] == 0
