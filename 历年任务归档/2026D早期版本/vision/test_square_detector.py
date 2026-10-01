"""
test_square_detector.py — SquareDetector 离线单元测试

使用合成图像，不依赖真实摄像头。
运行：python -m pytest vision/test_square_detector.py -v
"""

import numpy as np
import pytest
from .square_detector import SquareDetector, DetectionResult


def _make_frame(w: int, h: int, sq_cx: int, sq_cy: int, sq_half: int,
                bg: int = 200, fg: int = 0) -> np.ndarray:
    """生成灰度→BGR 的合成帧：浅灰背景 + 黑色实心方块。"""
    frame = np.full((h, w, 3), bg, dtype=np.uint8)
    x0 = max(0, sq_cx - sq_half)
    y0 = max(0, sq_cy - sq_half)
    x1 = min(w, sq_cx + sq_half)
    y1 = min(h, sq_cy + sq_half)
    frame[y0:y1, x0:x1] = fg
    return frame


# ─────────────────────────────────────────────────────────────────
# 基本检测
# ─────────────────────────────────────────────────────────────────

class TestBasicDetection:
    def test_center_square_found(self):
        """方块在画面正中，应该被检测到且中心坐标接近。"""
        det = SquareDetector()
        frame = _make_frame(320, 240, 160, 120, 40)
        result = det.detect(frame)
        assert result.found
        assert abs(result.cx_px - 160) < 10
        assert abs(result.cy_px - 120) < 10

    def test_off_center_square(self):
        """方块偏离中心，中心坐标应跟随。"""
        det = SquareDetector()
        frame = _make_frame(320, 240, 80, 60, 30)
        result = det.detect(frame)
        assert result.found
        assert abs(result.cx_px - 80) < 15
        assert abs(result.cy_px - 60) < 15

    def test_no_square_returns_not_found(self):
        """纯白帧，不应检测到任何目标。"""
        det = SquareDetector()
        frame = np.full((240, 320, 3), 220, dtype=np.uint8)
        result = det.detect(frame)
        assert not result.found

    def test_frame_dimensions_returned(self):
        """DetectionResult 应正确返回帧尺寸。"""
        det = SquareDetector()
        frame = _make_frame(640, 480, 320, 240, 60)
        result = det.detect(frame)
        assert result.frame_w == 640
        assert result.frame_h == 480


# ─────────────────────────────────────────────────────────────────
# 边界条件
# ─────────────────────────────────────────────────────────────────

class TestEdgeCases:
    def test_none_frame_returns_not_found(self):
        """传入 None 帧不应崩溃。"""
        det = SquareDetector()
        result = det.detect(None)
        assert not result.found

    def test_empty_frame_returns_not_found(self):
        """传入空 ndarray 不应崩溃。"""
        det = SquareDetector()
        result = det.detect(np.array([]))
        assert not result.found

    def test_too_close_flag(self):
        """方块占据画面 > 50%，应标记 too_close。"""
        det = SquareDetector(max_area_ratio=0.5)
        # 方块 200×200 在 320×240 画面中占 ~52%
        frame = _make_frame(320, 240, 160, 120, 100)
        result = det.detect(frame)
        assert result.found
        assert result.too_close

    def test_small_noise_ignored(self):
        """极小噪声方块（< min_area_ratio）不应被检测到。"""
        det = SquareDetector(min_area_ratio=0.01)
        # 4×4 像素方块在 320×240 中面积比 ≈ 0.0002，远低于 0.01
        frame = _make_frame(320, 240, 160, 120, 2)
        result = det.detect(frame)
        assert not result.found

    def test_area_ratio_range(self):
        """area_ratio 应在 (0, 1] 之间。"""
        det = SquareDetector()
        frame = _make_frame(320, 240, 160, 120, 50)
        result = det.detect(frame)
        if result.found:
            assert 0.0 < result.area_ratio <= 1.0
