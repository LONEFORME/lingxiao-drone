"""到达确认逻辑(滑动窗口N/M比例)的单元测试。

背景：2026-07-08矩形路径测试发现，旧逻辑要求"连续15帧"同时满足位置+速度
阈值，任何一帧不达标就把计数器清零重来；实测达标帧占比只有30-40%，导致
5个航点里3个都是靠arrival_timeout_max超时强制跳过，从未真正确认到达。
改成滑动窗口比例制后，偶发的单帧噪声不会清空已经积累的进度。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/circle_pole && python -m pytest test_arrival_confirm.py -v
"""
import os
import sys
from collections import deque

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Mission_GPT import (
    mission, arrival_window_confirmed, ARRIVAL_CONFIRM_RATIO,
    arrival_confirm_need,
)


# ════════════════════ arrival_window_confirmed (纯函数) ════════════════════

class TestArrivalWindowConfirmed:
    def test_window_not_full_never_confirms(self):
        window = deque([True] * 5, maxlen=arrival_confirm_need)
        assert arrival_window_confirmed(window, arrival_confirm_need, ARRIVAL_CONFIRM_RATIO) is False

    def test_full_window_all_true_confirms(self):
        window = deque([True] * arrival_confirm_need, maxlen=arrival_confirm_need)
        assert arrival_window_confirmed(window, arrival_confirm_need, ARRIVAL_CONFIRM_RATIO) is True

    def test_full_window_below_ratio_does_not_confirm(self):
        # 15帧里只有8帧达标(53.3%) < 60%阈值
        window = deque([True] * 8 + [False] * 7, maxlen=arrival_confirm_need)
        assert arrival_window_confirmed(window, arrival_confirm_need, ARRIVAL_CONFIRM_RATIO) is False

    def test_full_window_at_ratio_confirms(self):
        # 15帧里9帧达标(60%) == 阈值，应该确认
        window = deque([True] * 9 + [False] * 6, maxlen=arrival_confirm_need)
        assert arrival_window_confirmed(window, arrival_confirm_need, ARRIVAL_CONFIRM_RATIO) is True

    def test_single_noisy_frame_does_not_reset_progress(self):
        """核心场景：14帧达标里插1帧噪声(93%)，仍应该保持接近确认状态——
        跟旧的"严格连续"逻辑不同，这里不会因为1帧噪声就打回原点。"""
        window = deque([True] * 7 + [False] + [True] * 7, maxlen=arrival_confirm_need)
        assert arrival_window_confirmed(window, arrival_confirm_need, ARRIVAL_CONFIRM_RATIO) is True


# ════════════════════ navigate() 到达确认集成行为 ════════════════════

class FakeRealsenseArrival:
    def __init__(self, confidence=3, vel=(0.0, 0.0, 0.0)):
        self._confidence = confidence
        self._vel = vel

    def get_tracking_confidence(self):
        return self._confidence

    def get_velocity(self):
        return list(self._vel)


def _make_mission_at_target():
    re_fc = [0] * 14
    se_fc = [0] * 11
    m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None, radar_obj=None)
    m.t265_ok = True
    m.realsense = FakeRealsenseArrival(confidence=3, vel=(0.0, 0.0, 0.0))
    m.set_speed = lambda *a, **k: None
    return m


class TestNavigateArrivalConfirm:
    def test_one_noisy_frame_among_many_good_frames_still_confirms(self):
        """复现2026-07-08真机场景的简化版：位置/速度大多数时候达标，偶尔一帧
        速度尖峰超标。旧逻辑下这一帧会清零重来，永远凑不齐15帧连续；
        新逻辑下窗口比例仍然达标，应该能正常确认到达。"""
        m = _make_mission_at_target()
        target = m.targets[0]  # 默认航点[0,0,1.0]

        # 先跑13帧"在目标点、速度达标"的稳定帧
        for _ in range(13):
            m.navigate(list(target), 0.0)
        # 插1帧速度尖峰(0.07 > ARRIVAL_VEL_THRESH=0.05)
        m.realsense = FakeRealsenseArrival(confidence=3, vel=(0.07, 0.0, 0.0))
        m.navigate(list(target), 0.0)
        # 恢复稳定，再跑够窗口
        m.realsense = FakeRealsenseArrival(confidence=3, vel=(0.0, 0.0, 0.0))
        for _ in range(5):
            m.navigate(list(target), 0.0)

        assert m.arrival_confirmed_time is not None


# ═══════════════ arrival_hold_s / arrival_timeout_max 解耦 ═══════════════

class TestArrivalHoldTimeoutDecoupled:
    """2026-07-10曾把arrival_hold_s从3.0降到1.0想提速，结果反而更慢——因为
    当时arrival_timeout_max=5.0+arrival_hold_s是个耦合公式，缩短停留时间的
    同时也缩短了超时上限，导致滑动窗口更容易被截断、走更慢的超时兜底路径。
    2026-07-12把arrival_timeout_max改成独立常量，不再随arrival_hold_s变化，
    才能安全地单独压缩停留时间。"""

    def test_arrival_hold_s_is_0p3(self):
        """2026-07-12解耦arrival_timeout_max后先降到0.7秒真机验证有效
        (5/6航点确认，平均4.67秒/段，比历史基线快12-20%)。这次继续沿用
        同一条已验证的杠杆，进一步降到0.3秒。"""
        import Mission_GPT as mg
        assert mg.arrival_hold_s == 0.3

    def test_arrival_timeout_max_is_independent_constant_not_derived_from_hold_s(self):
        """arrival_timeout_max锁定在6.5(即改动前 5.0+1.5 的有效值)，不再是
        `5.0 + arrival_hold_s` 这个公式——如果之后有人不小心把arrival_hold_s
        改小，这条断言能防止arrival_timeout_max跟着被动缩短。"""
        import Mission_GPT as mg
        assert mg.arrival_timeout_max == 6.5

    def test_timeout_fires_at_arrival_timeout_max_not_derived_from_hold_s(self, monkeypatch):
        """行为验证：把arrival_timeout_max单独调成一个很短的值(0.1秒)，
        同时把arrival_hold_s调成一个很大的值(5.0秒，故意跟超时值方向相反)——
        如果两者还耦合，超时应该要等很久；解耦后超时只看arrival_timeout_max
        本身，应该在约0.1秒内就触发"超时强制跳过"。"""
        import Mission_GPT as mg
        monkeypatch.setattr(mg, "arrival_timeout_max", 0.1)
        monkeypatch.setattr(mg, "arrival_hold_s", 5.0)

        m = _make_mission_at_target()
        target = m.targets[0]
        far_pos = [target[0] + 10.0, target[1], target[2]]  # 远离目标，永远不会达标

        m.navigate(far_pos, 0.0)
        assert m.target_index == 0  # 刚开始，还没超时
        import time
        time.sleep(0.15)
        m.navigate(far_pos, 0.0)

        assert m.target_index == 1  # 超时已触发，强制跳到下一个航点
