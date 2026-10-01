import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

import numpy as np
import pytest

from Lcode.fire_vision import detect_fire, SmoothedFireDetector, FireVision, MIN_FIRE_AREA_PX, MAX_FIRE_AREA_PX


def _blank_frame(width=1920, height=1080):
    return np.zeros((height, width, 3), dtype=np.uint8)


def _draw_circle_bgr(frame, color_bgr, cx, cy, radius=40):
    cv2_circle(frame, color_bgr, cx, cy, radius)
    return frame


def cv2_circle(frame, color_bgr, cx, cy, radius):
    import cv2
    cv2.circle(frame, (cx, cy), radius, color_bgr, -1)


def _draw_rect_bgr(frame, color_bgr, cx, cy, half_w, half_h):
    y0, y1 = cy - half_h, cy + half_h
    x0, x1 = cx - half_w, cx + half_w
    frame[y0:y1, x0:x1] = color_bgr
    return frame


def _draw_starburst_bgr(frame, color_bgr, cx, cy, core_r, spike_len, n_spikes, thickness):
    """模拟强光源在画面里的"星芒/爆闪"高光形态：实心核心+若干条向外辐射的
    细尖刺，尖刺之间留有空隙——真实圆形光源(灯罩)在闭运算修复前经常被
    这种形态的轮廓算出很低的圆度(2026-07-17真机测试实测0.068)。"""
    import cv2
    cv2.circle(frame, (cx, cy), core_r, color_bgr, -1)
    for i in range(n_spikes):
        angle = 2 * np.pi * i / n_spikes
        x2 = int(cx + spike_len * np.cos(angle))
        y2 = int(cy + spike_len * np.sin(angle))
        cv2.line(frame, (cx, cy), (x2, y2), color_bgr, thickness)
    return frame


class TestDetectFire:
    def test_no_target_returns_none(self):
        frame = _blank_frame()
        result = detect_fire(frame)
        assert result is None

    def test_red_circle_detected_with_centered_offset_near_zero(self):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540, radius=40)
        dx_px, dy_px = detect_fire(frame)
        assert dx_px == pytest.approx(0.0, abs=5)
        assert dy_px == pytest.approx(0.0, abs=5)

    def test_offset_right_and_below_center_is_positive(self):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=1200, cy=700, radius=40)
        dx_px, dy_px = detect_fire(frame)
        assert dx_px > 0
        assert dy_px > 0

    def test_offset_left_and_above_center_is_negative(self):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=700, cy=300, radius=40)
        dx_px, dy_px = detect_fire(frame)
        assert dx_px < 0
        assert dy_px < 0

    def test_area_below_min_threshold_ignored(self):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540, radius=3)  # 面积远小于MIN_FIRE_AREA_PX
        assert detect_fire(frame) is None

    def test_area_above_max_threshold_ignored(self):
        """面积过大(比如整面墙反光)不当作火源，见设计文档审查发现的"误触发风险"应对。"""
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540, radius=500)  # 远超MAX_FIRE_AREA_PX
        assert detect_fire(frame) is None

    def test_elongated_rectangle_ignored_despite_valid_area(self):
        """2026-07-16真机测试实测：起点边上小车的红色矩形面积恰好落在合理范围内，
        被误判成火源触发了一次APPROACH。面积过滤挡不住"形状对但面积凑巧合理"的
        干扰物，需要额外用圆度过滤——真实火源灯罩俯视是近似圆形光斑，细长矩形
        (10:1长宽比)圆度远低于圆形，应该被拒绝。"""
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), cx=960, cy=540, half_w=200, half_h=20)  # 400x40，面积16000，10:1长宽比
        assert detect_fire(frame) is None

    def test_circle_with_similar_area_to_rejected_rectangle_still_detected(self):
        """对照组：跟上一个测试面积同量级(16000px)的圆形应该正常检测到，
        证明是形状过滤起作用，不是面积阈值本身变严格了。"""
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540, radius=71)  # pi*71^2约15837，同量级
        result = detect_fire(frame)
        assert result is not None

    def test_starburst_highlight_detected_after_mask_closing(self):
        """2026-07-17真机测试：灯罩强光在画面里经常呈"星芒/爆闪"形态(高光衍射
        产生多条细尖刺，不是实心圆盘)，导致轮廓圆度只有0.068，被MIN_CIRCULARITY
        挡住——排查确认不是曝光/距离问题，无论怎么调曝光，星芒状高光的圆度都
        上不去。用闭运算(FIRE_MASK_CLOSE_KERNEL_SIZE)把尖刺间的空隙填平再算
        圆度：这里用同等形态(实心核心+24条细尖刺，无闭运算时圆度0.65，低于0.7)
        复现，验证detect_fire()确实能检测到。"""
        frame = _blank_frame(width=960, height=540)
        _draw_starburst_bgr(frame, (0, 0, 255), cx=480, cy=270,
                             core_r=20, spike_len=25, n_spikes=24, thickness=7)
        result = detect_fire(frame)
        assert result is not None

    def test_occluded_fragment_shape_now_accepted_after_circularity_lowered(self):
        """2026-07-17真机测试：下视摄像头视场里的固定遮挡物(疑似脚架腿，已挪开)
        把一块红色矩形地标切成两个连通域，碎片圆度0.639曾混过0.5的阈值触发
        误判，当时把MIN_CIRCULARITY提到0.7挡住了它。但同一天晚些时候的短程
        近距离复测发现，真实火源近距离下圆度只有0.535~0.654，跟这块碎片的
        圆度区间直接重叠——圆度已经不能同时满足"识别真实火源"和"挡住这块
        碎片"，用户明确要求优先保证识别，阈值降回0.45。这里的200x80矩形
        (圆度约0.64)现在会被判定为疑似火源，不再是"防误判"用例，防线改为
        依赖"遮挡物已从现场物理挪开"这个根本修复，见known_issues.md。"""
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), cx=960, cy=540, half_w=100, half_h=40)  # 200x80，圆度约0.64
        assert detect_fire(frame) is not None


class TestSmoothedFireDetector:
    def test_no_detection_returns_none(self):
        det = SmoothedFireDetector(window=3)
        assert det.update(None) is None

    def test_single_frame_not_enough_returns_none(self):
        det = SmoothedFireDetector(window=3)
        assert det.update((10.0, 5.0)) is None

    def test_window_full_returns_average(self):
        det = SmoothedFireDetector(window=3)
        det.update((10.0, 0.0))
        det.update((20.0, 0.0))
        result = det.update((30.0, 0.0))
        assert result == pytest.approx((20.0, 0.0))

    def test_single_none_frame_resets_window(self):
        """检测中途丢失一帧应该重新积累，不能用陈旧帧掺进平均——避免真实火源移出
        画面后仍用旧偏移量继续修正(见设计文档APPROACH阶段防抖要求)。"""
        det = SmoothedFireDetector(window=3)
        det.update((10.0, 0.0))
        det.update((20.0, 0.0))
        det.update(None)  # 丢失一帧，窗口应清空
        assert det.update((5.0, 0.0)) is None  # 只有1帧，还不够


class _FakeVisionOpenFails:
    def __init__(self, **kwargs):
        pass

    def open(self):
        raise RuntimeError("Timed out waiting for the first camera frame")


class _FakeVisionOneFrame:
    """只在第一次get_frame()返回一帧红色图像，之后一直返回None，供测试用。"""

    def __init__(self, frame, **kwargs):
        self._frame = frame
        self._served = False
        self.released = False

    def open(self):
        return self

    def get_frame(self):
        if not self._served:
            self._served = True
            return self._frame.copy()
        return None

    def release(self):
        self.released = True


class TestFireVisionStartFailure:
    def test_start_returns_false_when_camera_open_raises(self, monkeypatch):
        """VisionSystem.open()打不开摄像头时抛RuntimeError(不是像
        cv2.VideoCapture.isOpened()那样返回False)——start()要把这个异常转换成
        统一的"返回False"约定，不能让main.py被这个异常打断。"""
        monkeypatch.setattr("Lcode.fire_vision.VisionSystem", _FakeVisionOpenFails)
        fv = FireVision()
        assert fv.start() is False

    def test_latest_before_any_frame_is_all_none(self):
        fv = FireVision()
        latest = fv.latest()
        assert latest["dx_px"] is None
        assert latest["dy_px"] is None
        assert latest["t"] == 0.0


class TestFireVisionBackgroundLoop:
    def test_loop_publishes_smoothed_detection_from_captured_frames(self, monkeypatch):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540)
        fake_vision = _FakeVisionOneFrame(frame)
        monkeypatch.setattr("Lcode.fire_vision.VisionSystem", lambda **kwargs: fake_vision)

        fv = FireVision(smooth_window=1)  # window=1: 单帧即可出结果，不用等多帧
        assert fv.start() is True
        deadline = time.time() + 2.0
        while fv.latest()["dx_px"] is None and time.time() < deadline:
            time.sleep(0.02)
        fv.stop()

        latest = fv.latest()
        assert latest["dx_px"] == pytest.approx(0.0, abs=5)
        assert latest["dy_px"] == pytest.approx(0.0, abs=5)
        assert latest["t"] > 0.0

    def test_stop_releases_camera(self, monkeypatch):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540)
        fake_vision = _FakeVisionOneFrame(frame)
        monkeypatch.setattr("Lcode.fire_vision.VisionSystem", lambda **kwargs: fake_vision)

        fv = FireVision(smooth_window=1)
        assert fv.start() is True
        deadline = time.time() + 2.0
        while fv.latest()["dx_px"] is None and time.time() < deadline:
            time.sleep(0.02)
        fv.stop()

        assert fake_vision.released is True

    def test_stop_joins_loop_thread_before_releasing(self, monkeypatch):
        """2026-07-17修复：真机测试main.py退出时实测到segfault——之前stop()
        只置位_running就立刻release()，_loop()线程可能还在执行get_frame()的
        过程中，跟release()拆卸底层采集资源产生竞态。stop()必须先join让轮询
        线程确认退出，才能安全release()。这里验证stop()返回后，_loop()那个
        线程对象确实已经不再存活。"""
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540)
        fake_vision = _FakeVisionOneFrame(frame)
        monkeypatch.setattr("Lcode.fire_vision.VisionSystem", lambda **kwargs: fake_vision)

        fv = FireVision(smooth_window=1)
        assert fv.start() is True
        deadline = time.time() + 2.0
        while fv.latest()["dx_px"] is None and time.time() < deadline:
            time.sleep(0.02)
        loop_thread = fv._thread
        fv.stop()

        assert loop_thread is not None
        assert loop_thread.is_alive() is False


class TestSaveSnapshot:
    """2026-07-16新增：火情触发时存一张现场画面方便事后调试分析。"""

    def test_returns_none_when_no_frame_available(self, tmp_path):
        fv = FireVision(debug_dir=str(tmp_path))
        assert fv.save_snapshot() is None

    def test_saves_latest_frame_and_returns_path(self, tmp_path, monkeypatch):
        frame = _blank_frame()
        _draw_circle_bgr(frame, (0, 0, 255), cx=960, cy=540)
        fake_vision = _FakeVisionOneFrame(frame)
        monkeypatch.setattr("Lcode.fire_vision.VisionSystem", lambda **kwargs: fake_vision)

        fv = FireVision(smooth_window=1, debug_dir=str(tmp_path))
        assert fv.start() is True
        deadline = time.time() + 2.0
        while fv.latest()["dx_px"] is None and time.time() < deadline:
            time.sleep(0.02)

        path = fv.save_snapshot(reason="fire_triggered")
        fv.stop()

        assert path is not None
        assert os.path.exists(path)
        assert "fire_triggered" in os.path.basename(path)

    def test_debug_dir_defaults_to_env_var(self, monkeypatch):
        monkeypatch.setenv("DRONE_FIRE_DEBUG_DIR", "/tmp/custom_fire_debug")
        fv = FireVision()
        assert fv.debug_dir == "/tmp/custom_fire_debug"
