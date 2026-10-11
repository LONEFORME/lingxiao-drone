"""
test_servo_controller.py — VisualServoController 状态机单元测试（v4）

使用 Detection 对象直接驱动，不依赖真实摄像头或 CyberCAM。
运行：cd drone_control/competition_2026 && python -m pytest vision/test_servo_controller.py -v
"""

import time

import pytest

from .servo_controller import Detection, ServoConfig, VisualServoController


# ─────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────

def _det(found=True, dx=0, dy=0, fw=320, fh=240):
    return Detection(found=found, dx_px=dx, dy_px=dy, frame_w=fw, frame_h=fh)


def _fast_cfg(**kw):
    defaults = dict(
        search_timeout_s=0.3,
        centering_timeout_s=1.0,
        centering_consec_frames=3,
        centering_threshold_m=0.30,
        focal_length_px=400.0,
        alt_stop_m=0.30,
        max_correction_cm_s=20.0,
        kp=1.0,
    )
    defaults.update(kw)
    return ServoConfig(**defaults)


# ─────────────────────────────────────────────────────────────────
# 高度截止：低于 alt_stop_m → done
# ─────────────────────────────────────────────────────────────────

class TestAltitudeStop:
    def test_below_threshold_returns_done(self):
        ctrl = VisualServoController(config=_fast_cfg())
        tick = ctrl.tick(_det(found=False), altitude_m=0.20)
        assert tick.done
        assert tick.reason == "alt_below_stop"

    def test_above_threshold_not_done(self):
        ctrl = VisualServoController(config=_fast_cfg())
        tick = ctrl.tick(_det(found=False), altitude_m=1.0)
        assert not tick.done
        assert not tick.failed


# ─────────────────────────────────────────────────────────────────
# SEARCHING 超时
# ─────────────────────────────────────────────────────────────────

class TestSearchTimeout:
    def test_not_found_times_out(self):
        ctrl = VisualServoController(
            config=_fast_cfg(search_timeout_s=0.1)
        )
        tick = _tick_repeated(ctrl, _det(found=False), timeout_s=0.5)
        assert tick.failed
        assert tick.reason == "search_timeout"

    def test_none_detection_times_out(self):
        ctrl = VisualServoController(
            config=_fast_cfg(search_timeout_s=0.1)
        )
        tick = _tick_repeated(ctrl, None, timeout_s=0.5)
        assert tick.failed


# ─────────────────────────────────────────────────────────────────
# CENTERING 正常收敛
# ─────────────────────────────────────────────────────────────────

class TestCenteringConvergence:
    def test_centered_detection_eventually_done(self):
        """dx=dy=0 连续 K 帧应返回 done=True。"""
        ctrl = VisualServoController(
            config=_fast_cfg(centering_consec_frames=3)
        )
        for _ in range(50):
            tick = ctrl.tick(_det(dx=0, dy=0), altitude_m=1.0)
            if tick.done or tick.failed:
                break
        assert tick.done, f"got failed={tick.failed} reason={tick.reason}"

    def test_off_center_returns_nonzero_velocity(self):
        """目标偏离中心，应返回非零速度修正。"""
        ctrl = VisualServoController(
            config=_fast_cfg(centering_consec_frames=100)
        )
        # 目标偏离 80px
        for _ in range(5):
            tick = ctrl.tick(_det(dx=80, dy=0), altitude_m=1.0)
        assert tick.state == "CENTERING"
        assert abs(tick.vy_cm_s) > 0.1, "Expected nonzero vy for dx=80"


# ─────────────────────────────────────────────────────────────────
# CENTERING 超时
# ─────────────────────────────────────────────────────────────────

class TestCenteringTimeout:
    def test_centering_timeout_returns_failed(self):
        """目标始终不满足阈值，超时应返回 failed。"""
        ctrl = VisualServoController(
            config=_fast_cfg(
                centering_timeout_s=0.2,
                centering_threshold_m=0.0001,  # 极小阈值：永不可能收敛
            )
        )
        tick = _tick_repeated(ctrl, _det(dx=80, dy=80), timeout_s=1.0)
        assert tick.failed
        assert tick.reason == "centering_timeout"


# ─────────────────────────────────────────────────────────────────
# reset()
# ─────────────────────────────────────────────────────────────────

class TestReset:
    def test_reset_clears_state(self):
        """reset() 后应重新从 SEARCHING 开始。"""
        ctrl = VisualServoController(config=_fast_cfg())
        # 推进到 CENTERING
        for _ in range(10):
            ctrl.tick(_det(dx=0, dy=0), altitude_m=1.0)
        # reset
        ctrl.reset()
        tick = ctrl.tick(_det(found=False), altitude_m=1.0)
        assert tick.state == "SEARCHING"


# ─────────────────────────────────────────────────────────────────
# 速度修正方向
# ─────────────────────────────────────────────────────────────────

class TestCorrectionDirection:
    def test_dx_positive_positive_vy(self):
        """dx>0（目标在右）→ vy > 0（向右修正）。"""
        ctrl = VisualServoController(
            config=_fast_cfg(centering_consec_frames=100)
        )
        for _ in range(5):
            tick = ctrl.tick(_det(dx=80, dy=0), altitude_m=1.0)
        if tick.state == "CENTERING":
            assert tick.vy_cm_s > 0, "dx>0 → vy should be positive"

    def test_dx_negative_negative_vy(self):
        """dx<0（目标在左）→ vy < 0（向左修正）。"""
        ctrl = VisualServoController(
            config=_fast_cfg(centering_consec_frames=100)
        )
        for _ in range(5):
            tick = ctrl.tick(_det(dx=-80, dy=0), altitude_m=1.0)
        if tick.state == "CENTERING":
            assert tick.vy_cm_s < 0, "dx<0 → vy should be negative"

    def test_dy_positive_positive_vx(self):
        """dy>0（目标在画面下方→在飞机前方）→ vx > 0（向前飞）。"""
        ctrl = VisualServoController(
            config=_fast_cfg(centering_consec_frames=100)
        )
        for _ in range(5):
            tick = ctrl.tick(_det(dx=0, dy=80), altitude_m=1.0)
        if tick.state == "CENTERING":
            assert tick.vx_cm_s > 0, "dy>0 → vx should be positive"


# ─────────────────────────────────────────────────────────────────
# Helper
# ─────────────────────────────────────────────────────────────────

def _tick_repeated(ctrl, det, timeout_s=1.0):
    """持续 tick() 直到 done/failed 或超时。"""
    tick = _no_tick()
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        tick = ctrl.tick(det, altitude_m=1.0)
        if tick.done or tick.failed:
            break
        time.sleep(0.02)
    return tick


def _no_tick():
    return type("S", (), {
        "done": False, "failed": False, "reason": "not_started"
    })()
