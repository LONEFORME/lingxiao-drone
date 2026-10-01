"""pole_vision 颜色检测纯函数单元测试。

运行:
    cd drone_control/circle_pole && python -m pytest test_pole_vision.py -v
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import cv2
import numpy as np
import pytest

from Lcode.pole_vision import detect_target, azimuth_from_dx, CAMERA_FOCAL_PX, CAMERA_FRAME_WIDTH


def _blank_frame(width=1920, height=1080):
    return np.zeros((height, width, 3), dtype=np.uint8)


def _draw_rect_bgr(frame, color_bgr, x_center, half_w=60, half_h=200):
    y_center = frame.shape[0] // 2
    frame[y_center - half_h:y_center + half_h,
          x_center - half_w:x_center + half_w] = color_bgr
    return frame


class TestDetectTarget:
    def test_no_target_returns_none_none(self):
        frame = _blank_frame()
        dx_px, color = detect_target(frame)
        assert dx_px is None
        assert color is None

    def test_red_rectangle_detected_as_red(self):
        frame = _blank_frame()
        # OpenCV红色HSV在色相环两端(0附近和180附近)，用纯红BGR(0,0,255)覆盖两段其中一段
        _draw_rect_bgr(frame, (0, 0, 255), x_center=960)
        dx_px, color = detect_target(frame)
        assert color == "red"
        assert dx_px == pytest.approx(0.0, abs=5)

    def test_green_rectangle_detected_as_green(self):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 255, 0), x_center=960)
        dx_px, color = detect_target(frame)
        assert color == "green"

    def test_dx_px_positive_when_target_right_of_center(self):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=1200)  # 画面中心960，目标在右侧
        dx_px, color = detect_target(frame)
        assert color == "red"
        assert dx_px > 0

    def test_dx_px_negative_when_target_left_of_center(self):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=700)
        dx_px, color = detect_target(frame)
        assert color == "red"
        assert dx_px < 0

    def test_colors_filter_ignores_unlisted_color(self):
        """颜色锁定用：只传锁定的颜色列表，画面里出现另一色也应该忽略。"""
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 255, 0), x_center=960)  # 只画绿色
        dx_px, color = detect_target(frame, colors=("red",))  # 只找红色
        assert dx_px is None
        assert color is None

    def test_area_below_min_threshold_ignored(self):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=960, half_w=2, half_h=2)  # 4x4像素，远小于阈值
        dx_px, color = detect_target(frame)
        assert dx_px is None
        assert color is None

    def test_flat_wide_blob_ignored_despite_large_area(self):
        """2026-07-15真机测试发现：场地天花板有一根绿色横梁，颜色跟真杆子
        完全重合(HSV阈值排查不掉)，但形状扁平(实测宽49~172×高12~159px)，
        真杆子是细高的(实测宽200×高507px)。纯色块检测(min_area)分不清两者，
        需要额外要求连通域的形状(高度+高宽比)像杆子。"""
        frame = _blank_frame()
        # 画一条宽600x高40的扁平绿色矩形(模拟横梁)，面积24000，远超min_area
        _draw_rect_bgr(frame, (0, 255, 0), x_center=960, half_w=300, half_h=20)
        dx_px, color = detect_target(frame)
        assert dx_px is None
        assert color is None

    def test_tall_pole_shape_preferred_over_larger_flat_blob(self):
        """同一帧里扁平横梁(面积更大，21000)和细高杆子(面积更小，13200)同时
        出现，应该选中杆子而不是面积更大的横梁——验证形状过滤优先于面积比较，
        不是"面积不够大的横梁恰好也会被面积阈值挡掉"这种巧合。"""
        frame = _blank_frame()
        # 横梁：宽700x高30，面积21000，画在画面上方，跟杆子不重叠
        frame[85:115, 200:900] = (0, 255, 0)
        # 杆子：宽60x高220，面积13200，比横梁面积小，画在画面下方
        _draw_rect_bgr(frame, (0, 255, 0), x_center=1300, half_w=30, half_h=110)
        dx_px, color = detect_target(frame)
        assert color == "green"
        assert dx_px == pytest.approx(1300 - 960, abs=5)  # 应该选中杆子(x_center=1300)，不是横梁(x_center=550)


class TestAzimuthFromDx:
    def test_zero_dx_is_zero_azimuth(self):
        assert azimuth_from_dx(0.0) == pytest.approx(0.0)

    def test_positive_dx_gives_positive_azimuth(self):
        assert azimuth_from_dx(500.0) > 0

    def test_negative_dx_gives_negative_azimuth(self):
        assert azimuth_from_dx(-500.0) < 0

    def test_matches_atan_formula_with_default_focal(self):
        dx = 300.0
        expected = math.atan(dx / CAMERA_FOCAL_PX)
        assert azimuth_from_dx(dx) == pytest.approx(expected)


import time

from Lcode.pole_vision import PoleVision


class _FakeCapOpenFails:
    def isOpened(self):
        return False


class _FakeCapOneFrame:
    """只在第一次read()返回一帧红色图像，之后一直返回失败，供测试用。"""

    def __init__(self, frame):
        self._frame = frame
        self._served = False
        self.released = False
        self.set_calls = []

    def isOpened(self):
        return True

    def read(self):
        if not self._served:
            self._served = True
            return True, self._frame
        return False, None

    def release(self):
        self.released = True

    def set(self, prop_id, value):
        self.set_calls.append((prop_id, value))
        return True


class TestPoleVisionStartFailure:
    def test_start_returns_false_when_camera_cannot_open(self, monkeypatch):
        monkeypatch.setattr("Lcode.pole_vision.cv2.VideoCapture",
                             lambda *_a, **_k: _FakeCapOpenFails())
        pv = PoleVision()
        assert pv.start() is False

    def test_latest_before_any_frame_is_all_none(self):
        pv = PoleVision()
        latest = pv.latest()
        assert latest["dx_px"] is None
        assert latest["color"] is None
        assert latest["t"] == 0.0


class TestPoleVisionResolution:
    def test_start_sets_1920x1080_mjpg_on_capture(self, monkeypatch):
        """2026-07-14真机测试发现：这颗USB摄像头默认给出320x240帧，而
        CAMERA_FOCAL_PX(1100)是按1920宽度标定的焦距，不显式设置分辨率会让
        azimuth_from_dx的换算完全错误(应该按帧宽等比例缩放焦距，而不是让帧
        宽跟标定值对不上)。start()必须在拿到能打开的capture后、起后台线程前，
        显式把格式/分辨率设成标定时用的1920x1080 MJPG。"""
        frame = _blank_frame()
        fake_cap = _FakeCapOneFrame(frame)
        monkeypatch.setattr("Lcode.pole_vision.cv2.VideoCapture", lambda *_a, **_k: fake_cap)

        pv = PoleVision()
        assert pv.start() is True
        pv.stop()

        prop_ids_set = [call[0] for call in fake_cap.set_calls]
        assert cv2.CAP_PROP_FOURCC in prop_ids_set
        assert cv2.CAP_PROP_FRAME_WIDTH in prop_ids_set
        assert cv2.CAP_PROP_FRAME_HEIGHT in prop_ids_set
        width_value = dict(fake_cap.set_calls)[cv2.CAP_PROP_FRAME_WIDTH]
        height_value = dict(fake_cap.set_calls)[cv2.CAP_PROP_FRAME_HEIGHT]
        assert width_value == CAMERA_FRAME_WIDTH
        assert height_value == 1080


class TestPoleVisionLockedColor:
    def test_set_locked_color_updates_state_and_is_readable(self):
        pv = PoleVision()
        assert pv._locked_color is None
        pv.set_locked_color("red")
        assert pv._locked_color == "red"
        pv.set_locked_color(None)
        assert pv._locked_color is None


class TestPoleVisionBackgroundLoop:
    def test_loop_publishes_detection_from_captured_frame(self, monkeypatch):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=960)
        fake_cap = _FakeCapOneFrame(frame)
        monkeypatch.setattr("Lcode.pole_vision.cv2.VideoCapture", lambda *_a, **_k: fake_cap)

        pv = PoleVision()
        assert pv.start() is True
        # 后台线程读一帧需要一点时间，轮询等待而不是固定sleep
        deadline = time.time() + 2.0
        while pv.latest()["color"] is None and time.time() < deadline:
            time.sleep(0.02)
        pv.stop()

        latest = pv.latest()
        assert latest["color"] == "red"
        assert latest["dx_px"] == pytest.approx(0.0, abs=5)
        assert latest["t"] > 0.0

    def test_stop_releases_camera_capture(self, monkeypatch):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=960)
        fake_cap = _FakeCapOneFrame(frame)
        monkeypatch.setattr("Lcode.pole_vision.cv2.VideoCapture", lambda *_a, **_k: fake_cap)

        pv = PoleVision()
        assert pv.start() is True
        deadline = time.time() + 2.0
        while pv.latest()["color"] is None and time.time() < deadline:
            time.sleep(0.02)
        pv.stop()


class TestPoleVisionDebugFrameSave:
    def test_saves_frame_when_debug_dir_set_and_color_detected(self, monkeypatch, tmp_path):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=960)
        fake_cap = _FakeCapOneFrame(frame)
        monkeypatch.setattr("Lcode.pole_vision.cv2.VideoCapture", lambda *_a, **_k: fake_cap)
        monkeypatch.setenv("DRONE_VISION_DEBUG_DIR", str(tmp_path))

        pv = PoleVision()
        assert pv.start() is True
        deadline = time.time() + 2.0
        while pv.latest()["color"] is None and time.time() < deadline:
            time.sleep(0.02)
        pv.stop()

        saved = list(tmp_path.glob("*_red.jpg"))
        assert len(saved) == 1

    def test_no_save_when_debug_dir_not_set(self, monkeypatch, tmp_path):
        frame = _blank_frame()
        _draw_rect_bgr(frame, (0, 0, 255), x_center=960)
        fake_cap = _FakeCapOneFrame(frame)
        monkeypatch.setattr("Lcode.pole_vision.cv2.VideoCapture", lambda *_a, **_k: fake_cap)
        monkeypatch.delenv("DRONE_VISION_DEBUG_DIR", raising=False)

        pv = PoleVision()
        assert pv.start() is True
        deadline = time.time() + 2.0
        while pv.latest()["color"] is None and time.time() < deadline:
            time.sleep(0.02)
        pv.stop()

        assert list(tmp_path.glob("*.jpg")) == []

        # release() 只会在后台线程下一次循环检测到 _running=False 后才调用，
        # 轮询等待而不是固定sleep或立即断言
        deadline = time.time() + 2.0
        while not fake_cap.released and time.time() < deadline:
            time.sleep(0.02)

        assert fake_cap.released is True


class _FakeCapMultiFrames:
    """依次返回给定的多帧图像，用完后read()一直失败，供测试用。"""

    def __init__(self, frames):
        self._frames = frames
        self._idx = 0
        self.released = False

    def isOpened(self):
        return True

    def read(self):
        if self._idx < len(self._frames):
            frame = self._frames[self._idx]
            self._idx += 1
            return True, frame
        return False, None

    def release(self):
        self.released = True

    def set(self, prop_id, value):
        return True


class TestPoleVisionDebugFrameSaveOnJump:
    def test_saves_frame_when_dx_px_jumps_without_color_change(self, monkeypatch, tmp_path):
        """真机测试发现：颜色锁定后(全程"green")dx_px仍会在两个相距很远的
        位置间瞬时跳变，怀疑detect_target()在两个不同的同色连通域间切换选中
        目标——颜色本身没变，之前"颜色切换才存图"的逻辑完全捕捉不到。"""
        frame1 = _blank_frame()
        _draw_rect_bgr(frame1, (0, 255, 0), x_center=200)  # 画面左侧
        frame2 = _blank_frame()
        _draw_rect_bgr(frame2, (0, 255, 0), x_center=1700)  # 画面右侧，同色但dx_px差很远
        fake_cap = _FakeCapMultiFrames([frame1, frame2])
        monkeypatch.setattr("Lcode.pole_vision.cv2.VideoCapture", lambda *_a, **_k: fake_cap)
        monkeypatch.setenv("DRONE_VISION_DEBUG_DIR", str(tmp_path))

        pv = PoleVision()
        assert pv.start() is True
        deadline = time.time() + 2.0
        while fake_cap._idx < 2 and time.time() < deadline:
            time.sleep(0.02)
        time.sleep(0.1)  # 等第二帧处理完写盘
        pv.stop()

        switch_saved = list(tmp_path.glob("*_green.jpg"))
        jump_saved = list(tmp_path.glob("*_green_jump.jpg"))
        assert len(switch_saved) == 1  # 第一帧：颜色从None切到green
        assert len(jump_saved) == 1    # 第二帧：颜色仍是green，但dx_px跳变超阈值

    def test_no_jump_save_when_dx_px_change_is_small(self, monkeypatch, tmp_path):
        frame1 = _blank_frame()
        _draw_rect_bgr(frame1, (0, 255, 0), x_center=960)
        frame2 = _blank_frame()
        _draw_rect_bgr(frame2, (0, 255, 0), x_center=1000)  # 只差40px，远小于阈值
        fake_cap = _FakeCapMultiFrames([frame1, frame2])
        monkeypatch.setattr("Lcode.pole_vision.cv2.VideoCapture", lambda *_a, **_k: fake_cap)
        monkeypatch.setenv("DRONE_VISION_DEBUG_DIR", str(tmp_path))

        pv = PoleVision()
        assert pv.start() is True
        deadline = time.time() + 2.0
        while fake_cap._idx < 2 and time.time() < deadline:
            time.sleep(0.02)
        time.sleep(0.1)
        pv.stop()

        jump_saved = list(tmp_path.glob("*_jump.jpg"))
        assert jump_saved == []
