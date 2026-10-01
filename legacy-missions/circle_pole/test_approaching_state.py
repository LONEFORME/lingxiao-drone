"""PATROL→APPROACHING触发滞回逻辑 + APPROACHING状态控制律单元测试。

运行:
    cd drone_control/circle_pole && python -m pytest test_approaching_state.py -v
"""
import io
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Mission_GPT import (
    mission, POLE_TRIGGER_CONFIRM_S, POLE_VISION_STALE_S,
    APPROACH_X_SPEED_FAR, APPROACH_X_SPEED_NEAR, APPROACH_CENTERED_DX_PX,
)
from Lcode.circle_planner import generate_circle_waypoints  # noqa: E402  (文件顶部已有sys.path.insert)


def _make_mission(radar_obj=None, pole_vision_obj=None):
    re_fc = [0] * 14
    se_fc = [0] * 11
    return mission(re_fc, se_fc, realsense_obj=None, serial_fc_ref=None,
                   radar_obj=radar_obj, pole_vision_obj=pole_vision_obj)


class _FakeVision:
    def __init__(self, dx_px=0.0, color="red", fresh=True):
        self._dx_px = dx_px
        self._color = color
        self._fresh = fresh
        self.locked_color = None

    def latest(self):
        t = time.time() if self._fresh else 0.0
        return {"dx_px": self._dx_px, "color": self._color, "t": t}

    def set_locked_color(self, color):
        self.locked_color = color


class TestPatrolTriggerIsVisionOnly(object):
    """2026-07-14现场测试后改为纯视觉触发：雷达有效探测距离(~1.2m)远小于
    摄像头，要求雷达+视觉同时确认等于要求飞机先物理飞到很近才能触发，起不到
    摄像头"远处先看到"的作用。雷达的职责后移到APPROACHING阶段内部(见
    test_approaching_state.py::TestApproachingRadarHandoff)。"""

    def test_no_vision_does_not_trigger(self):
        m = _make_mission(radar_obj=None, pole_vision_obj=None)
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"

    def test_vision_alone_does_not_trigger_before_confirm_window(self):
        """条件刚满足的第一帧不应该立刻触发，需要持续POLE_TRIGGER_CONFIRM_S。"""
        m = _make_mission(radar_obj=None, pole_vision_obj=_FakeVision(color="red"))
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"
        assert m._trigger_candidate == "red"

    def test_vision_alone_triggers_approaching_after_confirm_window_without_radar(self):
        """核心行为：不需要雷达确认，纯视觉持续看到颜色就能触发APPROACHING——
        雷达坐标此时还没有，_approach_pole_center应该是None，等
        _approaching_step内部轮询雷达后再冻结。"""
        m = _make_mission(radar_obj=None, pole_vision_obj=_FakeVision(color="red"))
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)  # 第一帧：只是记下候选，不触发
        assert m.nav_mode == "PATROL"

        m._trigger_candidate_since = time.time() - POLE_TRIGGER_CONFIRM_S - 0.01
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m.nav_mode == "APPROACHING"
        assert m._approach_pole_center is None
        assert m._approach_color == "red"

    def test_already_circled_color_does_not_trigger(self):
        m = _make_mission(radar_obj=None, pole_vision_obj=_FakeVision(color="red"))
        m.circled_colors.add("red")
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"
        assert m._trigger_candidate is None

    def test_stale_vision_does_not_trigger(self):
        m = _make_mission(radar_obj=None, pole_vision_obj=_FakeVision(color="red", fresh=False))
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"

    def test_color_change_resets_confirm_window(self):
        """候选颜色中途变化(比如视觉误判抖动)要重新计时，不能沿用旧计时器——
        这也是颜色锁定防抖机制的一部分，见2026-07-14设计文档"颜色锁定"一节。"""
        vision = _FakeVision(color="red")
        m = _make_mission(radar_obj=None, pole_vision_obj=vision)
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        first_since = m._trigger_candidate_since

        vision._color = "green"
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m._trigger_candidate == "green"
        assert m._trigger_candidate_since > first_since

    def test_off_center_vision_does_not_start_trigger_timer(self):
        """杆子颜色确认了但不在正前方(|dx_px|>=APPROACH_CENTERED_DX_PX)，
        不应该开始累积确认计时——巡航路线上杆子只是从画面边缘一闪而过时，
        不该打断巡航路线。"""
        vision = _FakeVision(color="red", dx_px=APPROACH_CENTERED_DX_PX + 50)
        m = _make_mission(radar_obj=None, pole_vision_obj=vision)
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"
        assert m._trigger_candidate is None
        assert m._trigger_candidate_since is None

    def test_becoming_centered_starts_timer_from_that_moment(self):
        """先偏离中心(不计时)，之后进入居中范围，确认计时器应该从"变成
        居中"这一刻才开始算，不是从颜色第一次出现那一刻算。"""
        vision = _FakeVision(color="red", dx_px=APPROACH_CENTERED_DX_PX + 50)
        m = _make_mission(radar_obj=None, pole_vision_obj=vision)
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m._trigger_candidate is None

        vision._dx_px = 0.0  # 变成居中
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m._trigger_candidate == "red"
        since_when_centered = m._trigger_candidate_since
        assert since_when_centered is not None

        # 计时未满0.3秒，不应该触发
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"

        # 计时满0.3秒后才应该触发
        m._trigger_candidate_since = time.time() - POLE_TRIGGER_CONFIRM_S - 0.01
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "APPROACHING"

    def test_drifting_off_center_mid_confirm_resets_timer(self):
        """确认窗口进行到一半时杆子偏出中心，应该导致这次确认作废，不能
        沿用之前累积的时长直接触发。"""
        vision = _FakeVision(color="red", dx_px=0.0)
        m = _make_mission(radar_obj=None, pole_vision_obj=vision)
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m._trigger_candidate == "red"

        # 手动把计时器往前拨，模拟已经过去了一段时间(但还没到0.3秒的边界)
        m._trigger_candidate_since = time.time() - (POLE_TRIGGER_CONFIRM_S - 0.05)

        vision._dx_px = APPROACH_CENTERED_DX_PX + 50  # 中途偏出中心
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"
        assert m._trigger_candidate is None
        assert m._trigger_candidate_since is None

        # 就算立刻重新居中，也必须从这一刻重新计满0.3秒，不能因为"之前已经
        # 攒了一部分时长"就提前触发
        vision._dx_px = 0.0
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert m.nav_mode == "PATROL"
        assert m._trigger_candidate == "red"


class TestApproachingControlLaw:
    def _start(self, pos, pole=(2.0, 0.0), color="red", dx_px=0.0):
        vision = _FakeVision(dx_px=dx_px, color=color)
        m = _make_mission(radar_obj=object(), pole_vision_obj=vision)
        m._approach_pole_center = pole
        m._approach_color = color
        m.nav_mode = "APPROACHING"
        # 2026-07-14 Task 13起_approaching_step也会轮询雷达(见悬停避让补充逻辑)，
        # 这里的fake radar(object())不支持get_scan()，需要让_last_pole_poll_time
        # 刚刚更新过，跳过本次轮询窗口，避免测试控制律的用例被雷达轮询意外打断——
        # 真实流程中nav_mode切到APPROACHING前PATROL分支刚做过一次轮询，效果等价。
        m._last_pole_poll_time = time.time()
        m.set_speed = lambda *a, **k: None
        m.navigate(pos, 0.0)
        return m

    def test_far_distance_uses_fast_speed(self):
        calls = []
        m = self._start(pos=[0.0, 0.0, 1.2], pole=(2.0, 0.0))
        m.set_speed = lambda x, y, yaw, z: calls.append(x)
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert calls[-1] == pytest.approx(APPROACH_X_SPEED_FAR)

    def test_near_distance_uses_slow_speed(self):
        calls = []
        m = self._start(pos=[1.25, 0.0, 1.2], pole=(2.0, 0.0))
        m.set_speed = lambda x, y, yaw, z: calls.append(x)
        m.navigate([1.25, 0.0, 1.2], 0.0)
        assert calls[-1] == pytest.approx(APPROACH_X_SPEED_NEAR)

    def test_approach_speed_direction_toward_pole(self):
        """杆塔在飞机后方(x更小)时，接近速度应该是负的(往回飞)。"""
        calls = []
        m = self._start(pos=[3.0, 0.0, 1.2], pole=(2.0, 0.0))
        m.set_speed = lambda x, y, yaw, z: calls.append(x)
        m.navigate([3.0, 0.0, 1.2], 0.0)
        assert calls[-1] < 0

    def test_vision_dx_drives_y_speed_via_pid(self):
        calls = []
        m = self._start(pos=[0.0, 0.0, 1.2], pole=(2.0, 0.0), dx_px=500.0)
        m.set_speed = lambda x, y, yaw, z: calls.append(y)
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert calls[-1] != 0

    def test_target_lost_beyond_timeout_retreats_to_patrol(self):
        stale_vision = _FakeVision(color="red", fresh=False)
        m = _make_mission(radar_obj=object(), pole_vision_obj=stale_vision)
        m._patrol_saved_targets = [[0.0, 0.0, 1.2], [1.0, 0.0, 1.2]]
        m._patrol_saved_index = 1
        m._approach_pole_center = (2.0, 0.0)
        m._approach_color = "red"
        m.nav_mode = "APPROACHING"
        m._last_pole_poll_time = time.time()  # 见TestApproachingControlLaw._start()注释
        m.set_speed = lambda *a, **k: None

        m.navigate([0.0, 0.0, 1.2], 0.0)  # 第一次陈旧：只是开始计时，不立刻退回
        assert m.nav_mode == "APPROACHING"

        m._approach_lost_since = time.time() - POLE_VISION_STALE_S - 0.01
        m._last_pole_poll_time = time.time()
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m.nav_mode == "PATROL"
        assert m.targets == [[0.0, 0.0, 1.2], [1.0, 0.0, 1.2]]
        assert m.target_index == 1
        assert m._approach_pole_center is None
        assert stale_vision.locked_color is None

    def test_reaching_trigger_distance_switches_to_circling(self):
        # 距离0.6m，在触发半径内——_start()内部的第一次navigate()调用就已经
        # 触发切换，不需要(也不能)再调用第二次navigate()：切换后nav_mode
        # 不再是APPROACHING，会走非APPROACHING分支并访问雷达scan接口，
        # 跟本测试用的fake radar(object())不兼容。
        m = self._start(pos=[2.1, 0.0, 1.2], pole=(2.7, 0.0))
        assert m.nav_mode == "CIRCLING"


class TestApproachingRadarHandoff:
    """2026-07-14现场测试后新增：APPROACHING触发时不再有雷达坐标
    (_approach_pole_center从None开始)，这里验证两件事——没有雷达坐标时的
    纯视觉控制律、以及雷达中途确认后正确切换到既有的距离阶梯控制。"""

    def _start_no_radar_center(self, pos, color="red", dx_px=0.0, radar_obj=None):
        vision = _FakeVision(dx_px=dx_px, color=color)
        m = _make_mission(radar_obj=radar_obj, pole_vision_obj=vision)
        m._approach_pole_center = None
        m._approach_color = color
        m.nav_mode = "APPROACHING"
        # 同TestApproachingControlLaw._start()的注释：fake radar(object())不支持
        # get_scan()，让_last_pole_poll_time刚更新过以跳过这次调用内的轮询窗口。
        m._last_pole_poll_time = time.time()
        m.set_speed = lambda *a, **k: None
        m.navigate(pos, 0.0)
        return m

    def test_not_centered_uses_zero_x_speed(self):
        calls = []
        m = self._start_no_radar_center(pos=[0.0, 0.0, 1.2], dx_px=APPROACH_CENTERED_DX_PX + 50)
        m.set_speed = lambda x, y, yaw, z: calls.append(x)
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert calls[-1] == 0

    def test_centered_uses_fixed_near_speed(self):
        calls = []
        m = self._start_no_radar_center(pos=[0.0, 0.0, 1.2], dx_px=APPROACH_CENTERED_DX_PX - 20)
        m.set_speed = lambda x, y, yaw, z: calls.append(x)
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert calls[-1] == pytest.approx(APPROACH_X_SPEED_NEAR)

    def test_no_radar_center_still_updates_y_via_vision(self):
        calls = []
        m = self._start_no_radar_center(pos=[0.0, 0.0, 1.2], dx_px=500.0)
        m.set_speed = lambda x, y, yaw, z: calls.append(y)
        m.navigate([0.0, 0.0, 1.2], 0.0)
        assert calls[-1] != 0

    def test_radar_confirms_mid_approach_freezes_center_and_switches_control(self):
        """雷达在APPROACHING期间(不是PATROL触发时)第一次确认出坐标，应该
        冻结_approach_pole_center并且不会把自己刚确认的这根杆塔误判成
        "其余未处理的杆塔"触发悬停(排除逻辑靠先冻结坐标再做悬停判断的顺序
        保证，见_approaching_step实现注释)。"""
        m = self._start_no_radar_center(pos=[0.0, 0.0, 1.2], dx_px=0.0, radar_obj=object())
        assert m._approach_pole_center is None

        # fake radar(object())不支持get_scan()：直接往_history里塞数据模拟
        # "已经轮询到"的结果，_last_pole_poll_time设成刚更新过以跳过真正调用
        # pole_tracker.update()(会打到不存在的radar.get_scan())。
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(2.0, 0.0)])
        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m._approach_pole_center == pytest.approx((2.0, 0.0))
        assert m._pole_hovering is False  # 没有被自己刚确认的目标误触发悬停

    def test_reaching_trigger_distance_after_radar_confirms_switches_to_circling(self):
        m = self._start_no_radar_center(pos=[2.1, 0.0, 1.2], dx_px=0.0, radar_obj=object())
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(2.7, 0.0)])  # 距离0.6m，在触发半径内
        m.set_speed = lambda *a, **k: None
        m.navigate([2.1, 0.0, 1.2], 0.0)

        assert m.nav_mode == "CIRCLING"


class TestStartCirclingFromApproach:
    def test_red_pole_circles_clockwise(self):
        m = _make_mission(radar_obj=object())
        m._approach_pole_center = (2.0, 0.0)
        m._approach_color = "red"
        m._cruise_z = 1.2

        m._start_circling_from_approach([1.3, 0.0, 1.2])

        assert m.nav_mode == "CIRCLING"
        expected = generate_circle_waypoints(2.0, 0.0, 1.3, 0.0, radius=0.7,
                                              n_points=6, direction="cw", z=1.2)
        assert m.targets == expected
        assert m.target_index == 0

    def test_green_pole_circles_counterclockwise(self):
        m = _make_mission(radar_obj=object())
        m._approach_pole_center = (2.0, 0.0)
        m._approach_color = "green"
        m._cruise_z = 1.2

        m._start_circling_from_approach([1.3, 0.0, 1.2])

        expected = generate_circle_waypoints(2.0, 0.0, 1.3, 0.0, radius=0.7,
                                              n_points=6, direction="ccw", z=1.2)
        assert m.targets == expected


class TestApproachingHoverAvoidance:
    def test_other_pole_triggers_hover_during_approaching(self):
        """设计文档承诺APPROACHING阶段悬停避让仍对"其余未处理的杆塔"生效——
        2026-07-14全量review发现原实现从未执行到这段逻辑(navigate()对APPROACHING
        直接return，跳过了雷达轮询/悬停避让代码块)。"""
        vision = _FakeVision(dx_px=0.0, color="red")
        m = _make_mission(radar_obj=object(), pole_vision_obj=vision)
        m._approach_pole_center = (5.0, 5.0)  # 正在接近的目标，离这次的杆子很远
        m._approach_color = "red"
        m.nav_mode = "APPROACHING"
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])  # 另一根杆子，离飞机只有0.3m

        m.set_speed = lambda *a, **k: None
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m._pole_hovering is True

    def test_own_approach_target_does_not_trigger_hover(self):
        vision = _FakeVision(dx_px=0.0, color="red")
        m = _make_mission(radar_obj=object(), pole_vision_obj=vision)
        m._approach_pole_center = (0.5, 0.3)
        m._approach_color = "red"
        m.nav_mode = "APPROACHING"
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.5, 0.3)])  # 就是接近目标本身

        m.set_speed = lambda *a, **k: None
        m.navigate([0.45, 0.28, 1.2], 0.0)

        assert m._pole_hovering is False


class TestApproachingTelemetryLogging:
    """2026-07-14真机测试观察到APPROACHING阶段"没有正对着杆子飞、靠近后来回
    震荡"的现象，排查时发现navigate()对APPROACHING直接return到_approaching_step，
    完全跳过了navigate()自己的飞行日志写入代码块——整个APPROACHING阶段(纯视觉
    接近+雷达接管后的距离阶梯控制)在flight_data.jsonl里没有留下任何位置/速度
    数据，没法验证/排查震荡现象。这里验证补上的_log_approaching_telemetry()
    确实被调用到。"""

    def test_pure_vision_phase_writes_log_entry(self):
        vision = _FakeVision(dx_px=0.0, color="red")
        m = _make_mission(radar_obj=None, pole_vision_obj=vision)
        m._approach_pole_center = None
        m._approach_color = "red"
        m.nav_mode = "APPROACHING"
        m._log_file = io.StringIO()
        m.set_speed = lambda *a, **k: None

        m.navigate([0.0, 0.0, 1.2], 0.0)

        m._log_file.seek(0)
        lines = [l for l in m._log_file.readlines() if l.strip()]
        assert len(lines) >= 1
        entry = json.loads(lines[-1])
        assert entry["nav_mode"] == "APPROACHING"
        assert entry["pos"] == [0.0, 0.0, 1.2]

    def test_radar_confirmed_phase_writes_log_entry_with_target(self):
        vision = _FakeVision(dx_px=0.0, color="red")
        m = _make_mission(radar_obj=None, pole_vision_obj=vision)
        m._approach_pole_center = (2.0, 0.5)
        m._approach_color = "red"
        m.nav_mode = "APPROACHING"
        m._log_file = io.StringIO()
        m.set_speed = lambda *a, **k: None

        m.navigate([0.0, 0.0, 1.2], 0.0)

        m._log_file.seek(0)
        entry = json.loads([l for l in m._log_file.readlines() if l.strip()][-1])
        assert entry["target"][0] == pytest.approx(2.0)
        assert entry["target"][1] == pytest.approx(0.5)

    def test_hover_avoidance_phase_writes_log_entry(self):
        vision = _FakeVision(dx_px=0.0, color="red")
        m = _make_mission(radar_obj=object(), pole_vision_obj=vision)
        m._approach_pole_center = (5.0, 5.0)
        m._approach_color = "red"
        m.nav_mode = "APPROACHING"
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])
        m._log_file = io.StringIO()
        m.set_speed = lambda *a, **k: None

        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert m._pole_hovering is True
        m._log_file.seek(0)
        entry = json.loads([l for l in m._log_file.readlines() if l.strip()][-1])
        assert entry["pole_hover"] is True

    def test_hovering_blocks_approach_speed_commands(self):
        """悬停期间不应该继续输出接近速度指令(vx朝目标杆)，应该是悬停锁定
        位置的PID输出。"""
        vision = _FakeVision(dx_px=0.0, color="red")
        m = _make_mission(radar_obj=object(), pole_vision_obj=vision)
        m._approach_pole_center = (5.0, 5.0)
        m._approach_color = "red"
        m.nav_mode = "APPROACHING"
        m._last_pole_poll_time = time.time()
        for _ in range(3):
            m.pole_tracker._history.append([(0.3, 0.0)])

        calls = []
        m.set_speed = lambda x, y, yaw, z: calls.append(x)
        m.navigate([0.0, 0.0, 1.2], 0.0)

        assert calls[-1] != APPROACH_X_SPEED_FAR
        assert calls[-1] != APPROACH_X_SPEED_NEAR
