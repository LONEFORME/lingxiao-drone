"""PoleTracker接入Mission_GPT导航流程的单元测试。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/circle_pole && python -m pytest test_mission_pole_integration.py -v
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Mission_GPT import mission, nearest_confirmed_pole_dist, POLE_DANGER_DIST_M, POLE_RESUME_DIST_M


# ════════════════════ nearest_confirmed_pole_dist ════════════════════

class TestNearestConfirmedPoleDist:
    def test_empty_list_returns_none(self):
        assert nearest_confirmed_pole_dist([], 0.0, 0.0) is None

    def test_single_pole_returns_its_distance(self):
        poles = [{"x": 0.3, "y": 0.0, "hits": 3}]
        assert nearest_confirmed_pole_dist(poles, 0.0, 0.0) == pytest.approx(0.3)

    def test_multiple_poles_returns_nearest(self):
        poles = [
            {"x": 5.0, "y": 0.0, "hits": 3},
            {"x": 0.4, "y": 0.3, "hits": 3},  # 距(0,0) = 0.5
        ]
        assert nearest_confirmed_pole_dist(poles, 0.0, 0.0) == pytest.approx(0.5)


# ════════════════════ mission 悬停/恢复行为 ════════════════════
#
# circle_pole特有说明(2026-07-13，两次修正)：
# 第一次修正：PATROL态第一次发现一个"未环绕过"的确认杆塔时，行为从"悬停"改成
# "立即开始环绕"(见test_circle_state_machine.py)。
# 第二次修正(真机测试发现)：已经环绕完成的杆塔，之后也**不再**触发悬停避让——
# 真机测试发现环绕刚完成那一刻飞机必然还在环绕半径(0.7m)内，小于悬停触发阈值
# (0.75m)，如果"已绕过的杆塔"恢复正常悬停判断，会在切换到TO_LANDING的瞬间被
# 自己刚绕完的杆塔悬停住，直到POLE_HOVER_TIMEOUT_S(15秒)超时强制原地降落，
# 不管降落点方向选得对不对都会复现。所以已环绕杆塔现在被永久排除在悬停判断外。
# 第三次修正(2026-07-14设计文档"CIRCLING阶段悬停避让整体关闭"一节)：CIRCLING
# 阶段悬停避让整体关闭，不只排除当前环绕目标——理论安全边际(5cm)被定位噪声
# 完全吞掉，环绕中途悬停打断本身比不检测风险更高。
# 悬停避让在circle_pole里唯一还有效的场景是：非CIRCLING阶段(PATROL/APPROACHING/
# TO_LANDING/RETREAT)下，遇到另一根**不是**当前接近目标、也**没**环绕过的杆塔
# (阶段2双杆场景)。下面几个测试用TO_LANDING态而非PATROL态来触发这个场景：
# PATROL态下navigate()仍会先跑旧的_find_new_pole+_start_circling直接触发逻辑
# (Task 7才替换成APPROACHING态的视觉触发流程)，任何确认杆塔都会被当成"新目标"
# 立即环绕，走不到悬停判断。TO_LANDING没有这类早期分支，是稳定的选择。

def _make_mission(radar_obj=None):
    re_fc = [0] * 14
    se_fc = [0] * 11
    return mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None, radar_obj=radar_obj)


class TestMissionPoleHover:
    def test_no_radar_means_no_pole_tracker_and_no_hover(self):
        m = _make_mission(radar_obj=None)
        assert m.pole_tracker is None
        m.navigate([0.0, 0.0, 1.0], 0.0)  # 不应该抛异常
        assert m._pole_hovering is False

    def test_navigate_does_not_hover_when_already_circled_pole_confirmed_nearby(self):
        """2026-07-13真机测试发现的回归测试：已环绕过的杆塔重新靠近也不应该悬停
        （旧版本这里会悬停，导致环绕完成后原地卡住直到15秒超时强制降落）。"""
        m = _make_mission(radar_obj=object())  # 哨兵对象，本测试跳过真实轮询，不会被调用
        m.circled_poles = [(0.3, 0.0, "red")]  # 已环绕过
        m._last_pole_poll_time = time.time()  # 跳过本帧的雷达轮询(节流)，直接摆好历史数据
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])  # 世界坐标(0.3,0)，离(0,0)只有0.3m

        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.0], 0.0)

        assert m._pole_hovering is False

    def test_navigate_does_not_hover_when_already_circled_pole_position_has_drifted(self):
        """2026-07-13真机测试发现的第二个回归测试：confirmed_poles()每次都从滑动
        窗口重新平均杆塔世界坐标，同一根已环绕过的静止杆塔的估计位置会随窗口滚动
        逐渐漂移。用比POLE_WORLD_MATCH_EPS_M(0.2m)更宽的容差(环绕半径+余量)判断
        "是否已环绕过"，否则漂移超过0.2m就会被误判成新杆塔，重新触发悬停。"""
        m = _make_mission(radar_obj=object())
        m.circled_poles = [(-0.91, -0.28, "red")]  # 已环绕过的杆塔记录位置
        m._last_pole_poll_time = time.time()
        # 当前重新计算出的位置漂移了约0.41m(超过0.2m旧容差，但在1.1m新容差内)
        for _ in range(3):
            m.pole_tracker._history.append([(-0.5, -0.28)])

        m.set_speed = lambda *a, **k: None
        m.navigate([0.06, -0.12, 1.0], 0.0)  # 飞机当前位置离漂移后的坐标很近

        assert m._pole_hovering is False

    def test_navigate_does_not_hover_when_pole_far_away(self):
        m = _make_mission(radar_obj=object())
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(5.0, 5.0)])  # 远超 POLE_DANGER_DIST_M

        m.navigate([0.0, 0.0, 1.0], 0.0)

        assert m._pole_hovering is False

    def test_navigate_hovers_for_different_uncircled_pole_outside_circling(self):
        """悬停避让唯一还有效的场景：非CIRCLING阶段(PATROL/APPROACHING/TO_LANDING/
        RETREAT)下遇到一根不是当前接近目标、也没环绕过的杆塔(阶段2双杆场景的安全
        网)。CIRCLING阶段整体关闭悬停避让(2026-07-14设计文档"CIRCLING阶段悬停
        避让整体关闭"一节)，不再是这里验证的场景。
        用TO_LANDING态而非PATROL态来触发：PATROL态下`navigate()`仍会先跑旧的
        `_find_new_pole`+`_start_circling`直接触发逻辑(Task 7才会替换成
        APPROACHING态的视觉触发流程)，任何确认杆塔(不区分远近/是否是接近目标)
        都会被当成"新目标"立即开始环绕，根本走不到悬停避让判断就return了。
        TO_LANDING没有这类早期分支(Task 8也只拦截APPROACHING态)，是稳定的选择。"""
        m = _make_mission(radar_obj=object())
        m._approach_pole_center = (5.0, 5.0)  # 正在接近的目标，离这次的杆子很远
        m.nav_mode = "TO_LANDING"
        m.targets = [[5.0, 4.3, 1.0]]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])  # 另一根杆子，离飞机只有0.3m

        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.0], 0.0)

        assert m._pole_hovering is True

    def test_navigate_keeps_hovering_within_resume_hysteresis_band(self):
        """已经悬停时，杆子距离哪怕超过了触发阈值(POLE_DANGER_DIST_M)，只要还没到
        恢复阈值(POLE_RESUME_DIST_M)，应该继续悬停——避免距离刚好卡在触发阈值附近
        抖动时悬停状态跟着反复横跳(2026-07-08真机测试观察到的问题)。用TO_LANDING
        态+另一根不相关的杆子来触发(见上一个测试的说明，PATROL态下会被旧的
        `_find_new_pole`直接触发逻辑抢先拦截，走不到悬停判断)。"""
        m = _make_mission(radar_obj=object())
        m._approach_pole_center = (5.0, 5.0)
        m.nav_mode = "TO_LANDING"
        m.targets = [[5.0, 4.3, 1.0]]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.0], 0.0)
        assert m._pole_hovering is True

        # 飞机远离杆子，让实际距离变成触发/恢复阈值之间的中点：超过触发阈值但还没到
        # 恢复阈值，应该继续悬停。杆子世界坐标保持不变，用飞机自身位置远离来改变
        # 距离，才是真机场景下距离变化的真实成因。
        # (2026-07-10阈值从0.9/1.05回退到0.75/0.9，见CLAUDE.md问题20——用常量算中点
        # 而不是硬编码具体数字，避免下次再调阈值时这个测试又悄悄失效)
        mid_dist = (POLE_DANGER_DIST_M + POLE_RESUME_DIST_M) / 2
        m.pole_tracker.reset()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])
        m._last_pole_poll_time = time.time()
        m.navigate([0.3 - mid_dist, 0.0, 1.0], 0.0)
        assert m._pole_hovering is True

    def test_navigate_resumes_once_pole_dist_exceeds_resume_threshold(self):
        """非CIRCLING阶段(TO_LANDING)下悬停恢复行为(CIRCLING阶段已整体关闭悬停
        避让)。选TO_LANDING理由同上一个测试(PATROL态会被旧的_find_new_pole直接
        触发逻辑抢先拦截)。"""
        m = _make_mission(radar_obj=object())
        m._approach_pole_center = (5.0, 5.0)
        m.nav_mode = "TO_LANDING"
        m.targets = [[5.0, 4.3, 1.0]]
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.0], 0.0)
        assert m._pole_hovering is True

        # 飞机远离杆子到实际距离1.1m：超过恢复阈值(0.9)，应该恢复导航。
        # 同上一个测试的说明：杆子世界坐标保持不变，靠飞机位置远离来改变距离。
        m.pole_tracker.reset()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])
        m._last_pole_poll_time = time.time()
        m.navigate([0.3 - 1.1, 0.0, 1.0], 0.0)
        assert m._pole_hovering is False
