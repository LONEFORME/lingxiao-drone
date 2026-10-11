"""到达确认逻辑(滑动窗口N/M比例)的单元测试。

背景：2026-07-08矩形路径测试发现，旧逻辑要求"连续15帧"同时满足位置+速度
阈值，任何一帧不达标就把计数器清零重来；实测达标帧占比只有30-40%，导致
5个航点里3个都是靠arrival_timeout_max超时强制跳过，从未真正确认到达。
改成滑动窗口比例制后，偶发的单帧噪声不会清空已经积累的进度。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/basic_radar && python -m pytest test_arrival_confirm.py -v
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
    m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None)
    # 本夹具只验证到达窗口，不提供飞控串口反馈；显式使用旧 T265 航向源，
    # 避免默认 FC 航向源的安全门禁掩盖被测导航行为。
    m.heading_source = "t265"
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
