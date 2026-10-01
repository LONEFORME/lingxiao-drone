"""PATROL/CIRCLING/TO_LANDING状态机单元测试。

运行:
    cd drone_control/circle_pole && python -m pytest test_circle_state_machine.py -v
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Mission_GPT import mission, POLE_CIRCLE_N_POINTS


def _make_mission(radar_obj=None, pole_total=1):
    re_fc = [0] * 14
    se_fc = [0] * 11
    m = mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None, radar_obj=radar_obj)
    m.pole_total = pole_total
    return m


class TestColorDedup:
    def test_already_circled_color_is_not_in_circled_colors_initially(self):
        m = _make_mission(radar_obj=object())
        assert m.circled_colors == set()

    def test_color_already_circled_true_after_recorded(self):
        m = _make_mission(radar_obj=object())
        m.circled_colors.add("red")
        assert m._color_already_circled("red") is True
        assert m._color_already_circled("green") is False


class TestCirclingHoverExclusion:
    def test_own_circling_target_does_not_trigger_hover(self):
        m = _make_mission(radar_obj=object())
        m._approach_pole_center = (0.5, 0.3)
        m.nav_mode = "CIRCLING"
        m.targets = [[0.5, -0.2, 1.2], [1.0, 0.3, 1.2]]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])  # 就是环绕目标本身

        m.set_speed = lambda *a, **k: None
        m.navigate([0.45, 0.28, 1.2], 0.0)

        assert m._pole_hovering is False

    def test_circling_ignores_any_other_pole_regardless_of_distance(self):
        """2026-07-14决定：CIRCLING阶段悬停避让整体关闭，不只排除当前目标。哪怕
        另一根完全不相关的杆子就在飞机旁边(0.1m，远小于悬停阈值)，环绕过程中也
        不应该被打断——赛题最小间距150cm+环绕半径0.7m下，理论安全边际只有5cm，
        被已知定位噪声完全吞掉，环绕中途悬停打断本身比不检测风险更高。"""
        m = _make_mission(radar_obj=object())
        m._approach_pole_center = (5.0, 5.0)  # 正在环绕的目标，离这次的杆子很远
        m.nav_mode = "CIRCLING"
        m.targets = [[5.0, 4.3, 1.2], [5.0, 5.7, 1.2]]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.1, 0.0)])  # 另一根杆子，离飞机只有0.1m

        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m._pole_hovering is False

    def test_own_circling_target_still_excluded_despite_yaw_drift_offset(self):
        """环绕过程中yaw漂移可能让雷达对同一根杆子的世界坐标算出偏移，验证排除逻辑
        用的是环绕半径+余量的宽松容差(POLE_CIRCLE_EXCLUDE_MARGIN_M)，不是原本给
        "同一物体"判断设计的0.2m窄容差——否则漂移会让排除逻辑"认不出"自己正在
        环绕的目标，中途误触发悬停(见2026-07-13讨论的风险1)。"""
        m = _make_mission(radar_obj=object())
        m._approach_pole_center = (0.5, 0.3)
        m.nav_mode = "CIRCLING"
        m.targets = [[0.5, -0.2, 1.2], [1.0, 0.3, 1.2]]
        m._last_pole_poll_time = time.time()
        # 偏移0.3m(超过旧的0.2m容差，但在新的排除半径1.1m以内)
        for _ in range(3):
            m.pole_tracker._history.append([(0.8, 0.3)])

        m.set_speed = lambda *a, **k: None
        m.navigate([0.75, 0.28, 1.2], 0.0)

        assert m._pole_hovering is False


class TestCircleCompletion:
    def test_circle_done_retreats_to_x_zero_baseline_first(self):
        m = _make_mission(radar_obj=object(), pole_total=1)
        m.nav_mode = "CIRCLING"
        m._approach_pole_center = (0.5, 0.3)
        m._approach_color = "red"
        m.targets = [[0.5, 0.8, 1.2]]
        m.target_index = 1  # 已飞完最后一个环绕航点

        m.navigate([0.5, 0.8, 1.2], 0.0)

        assert m.nav_mode == "RETREAT"
        assert m.circled_poles == [(0.5, 0.3, "red")]
        assert m.circled_colors == {"red"}
        assert m.targets == [[0.0, 0.8, m._cruise_z]]
        assert m.target_index == 0
        assert m._approach_pole_center is None

    def test_single_color_mission_switches_to_to_landing_after_retreat(self):
        m = _make_mission(radar_obj=object(), pole_total=1)
        m.nav_mode = "RETREAT"
        m.circled_colors = {"red"}
        m.targets = [[0.0, 0.8, 1.2]]
        m.target_index = 1  # 已到达基准线

        m.navigate([0.0, 0.8, 1.2], 0.0)

        assert m.nav_mode == "TO_LANDING"
        assert m.target_index == 0
        assert len(m.targets) >= 1  # 从landing_router.txt加载

    def test_retreat_to_to_landing_resets_detour_checked_index(self):
        """review发现的潜在gap：RETREAT的唯一航点(index 0)已经跑过一次
        _maybe_insert_detour并把_detour_checked_index设成0；如果进入TO_LANDING时
        不重置，TO_LANDING第一个航点同样是index 0，会被_maybe_insert_detour误判
        成"已经检查过"而静默跳过绕行检测——跟_start_circling_from_approach进入
        CIRCLING时显式重置_detour_checked_index=-1是同一类问题。"""
        m = _make_mission(radar_obj=object(), pole_total=1)
        m.nav_mode = "RETREAT"
        m.circled_colors = {"red"}
        m.targets = [[0.0, 0.8, 1.2]]
        m.target_index = 1  # 已到达基准线
        m._detour_checked_index = 0  # 模拟RETREAT航点已经跑过绕行检测

        m.navigate([0.0, 0.8, 1.2], 0.0)

        assert m.nav_mode == "TO_LANDING"
        assert m._detour_checked_index == -1

    def test_multi_color_mission_resumes_patrol_after_retreat(self):
        m = _make_mission(radar_obj=object(), pole_total=2)
        patrol_targets = [[0.0, 0.0, 1.2], [1.0, 0.0, 1.2], [2.0, 0.0, 1.2]]
        m._patrol_saved_targets = patrol_targets
        m._patrol_saved_index = 1
        m.nav_mode = "RETREAT"
        m.circled_colors = {"red"}  # 只绕完一种颜色，还差一种
        m.targets = [[0.0, 0.8, 1.2]]
        m.target_index = 1

        m.navigate([0.0, 0.8, 1.2], 0.0)

        assert m.nav_mode == "PATROL"
        assert m.targets == patrol_targets
        assert m.target_index == 1

    def test_to_landing_arrival_transitions_to_land_state(self):
        m = _make_mission(radar_obj=None)
        m.nav_mode = "TO_LANDING"
        m.targets = [[2.0, 0.0, 1.2]]
        m.target_index = 1  # 已到达唯一的降落航点

        m.navigate([2.0, 0.0, 1.2], 0.0)

        assert m.state == "LAND"


class TestDetourInsertion:
    def test_inserts_detour_when_already_circled_pole_blocks_direct_path(self):
        """2026-07-13讨论的场景：已经环绕过的杆塔从悬停避让里永久排除了，如果
        巡航/降落路径又正好直飞穿过它，绕行是唯一还能防碰撞的机制。"""
        m = _make_mission(radar_obj=object())
        m.circled_poles = [(1.0, 0.0, "red")]  # 已环绕过，挡在(0,0)->(2,0)正中间
        m.nav_mode = "PATROL"
        m.targets = [[2.0, 0.0, 1.2]]
        m.target_index = 0
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(1.0, 0.0)])

        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert len(m.targets) == 2
        detour = m.targets[0]
        assert m.targets[1] == [2.0, 0.0, 1.2]  # 原目标还在，往后挪了一位
        # 绕行点应该在安全半径(0.75)+余量(0.15)之外
        import math
        dist = math.hypot(detour[0] - 1.0, detour[1] - 0.0)
        assert dist == pytest.approx(0.75 + 0.15, abs=1e-6)

    def test_does_not_insert_detour_for_actively_circling_target(self):
        """正在主动环绕的目标不应该触发绕行检测——环绕航点本身就是绕着它走的。"""
        m = _make_mission(radar_obj=object())
        m._approach_pole_center = (1.0, 0.0)
        m.nav_mode = "CIRCLING"
        m.targets = [[2.0, 0.0, 1.2]]
        m.target_index = 0
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(1.0, 0.0)])

        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert len(m.targets) == 1  # 没有插入绕行航点

    def test_does_not_recheck_same_target_index_repeatedly(self):
        """同一个target_index只检查一次，不会每tick重复插入绕行航点。"""
        m = _make_mission(radar_obj=object())
        m.circled_poles = [(1.0, 0.0, "red")]
        m.nav_mode = "PATROL"
        m.targets = [[2.0, 0.0, 1.2]]
        m.target_index = 0
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(1.0, 0.0)])

        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert len(m.targets) == 2

        m.navigate([0.0, 0.0, 1.2], 0.0)  # 再调用一次，target_index还是0(指向绕行点)
        assert len(m.targets) == 2  # 没有重复插入

    def test_does_not_insert_detour_during_circling_even_for_already_circled_pole(self):
        """2026-07-14全量review发现：CIRCLING阶段绕行检测之前没有整体关闭，只排除
        当前正在环绕的目标，不排除已环绕过的杆塔——如果环绕路径途经已环绕杆塔的
        安全区，会被插入绕行航点，打断环绕路径本身，跟设计文档"绕行机制发生在
        PATROL/TO_LANDING态，不受CIRCLING阶段影响"的说法矛盾。"""
        m = _make_mission(radar_obj=object())
        m.circled_poles = [(1.0, 0.0, "red")]  # 已环绕过，挡在这次的环绕路径中间
        m._approach_pole_center = (5.0, 5.0)  # 正在环绕的另一个目标(远处，不相关)
        m.nav_mode = "CIRCLING"
        m.targets = [[2.0, 0.0, 1.2]]
        m.target_index = 0
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(1.0, 0.0)])

        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert len(m.targets) == 1  # 没有插入绕行航点，CIRCLING阶段完全不做绕行检测
