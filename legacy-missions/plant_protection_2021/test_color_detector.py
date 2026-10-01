"""测试颜色识别模块。"""
import numpy as np
import pytest

from Lcode.color_detector import (
    ColorDetector,
    ColorDetectorConfig,
    GridColor,
)
from Lcode.video_source import VideoFrame


# ── 工厂 ──────────────────────────────────────────────

def _make_bgr_frame(
    width: int = 320,
    height: int = 240,
    bgr: tuple[int, int, int] = (0, 255, 0),
) -> VideoFrame:
    """创建纯色 BGR 帧。"""
    payload = np.full((height, width, 3), bgr, dtype=np.uint8)
    return VideoFrame(
        sequence=0,
        captured_at=0.0,
        width=width,
        height=height,
        pixel_format="bgr24",
        payload=payload,
    )


def _make_checkerboard_frame(
    width: int = 320,
    height: int = 240,
    block_size: int = 40,
    color_a: tuple = (0, 255, 0),    # 绿
    color_b: tuple = (240, 240, 240),  # 灰
) -> VideoFrame:
    """创建棋盘格帧，用于测试 ROI 切割和混合场景。"""
    payload = np.zeros((height, width, 3), dtype=np.uint8)
    for y in range(0, height, block_size):
        for x in range(0, width, block_size):
            color = color_a if ((x // block_size) + (y // block_size)) % 2 == 0 else color_b
            payload[y:y+block_size, x:x+block_size] = color
    return VideoFrame(
        sequence=0,
        captured_at=0.0,
        width=width,
        height=height,
        pixel_format="bgr24",
        payload=payload,
    )


# ── 测试用例 ──────────────────────────────────────────

@pytest.mark.fast
def test_green_frame_classifies_as_green():
    """全绿帧应识别为 GREEN。"""
    det = ColorDetector()
    det.set_frame(_make_bgr_frame(bgr=(0, 255, 0)))  # 纯绿
    color, conf = det.classify()
    assert color == GridColor.GREEN
    assert conf >= 0.9


@pytest.mark.fast
def test_gray_frame_classifies_as_gray():
    """全灰帧应识别为 GRAY。"""
    det = ColorDetector()
    det.set_frame(_make_bgr_frame(bgr=(240, 240, 240)))  # 浅灰
    color, conf = det.classify()
    assert color == GridColor.GRAY
    assert conf >= 0.9


@pytest.mark.fast
def test_red_frame_returns_unknown():
    """红色帧（非绿非灰）应返回 UNKNOWN。"""
    det = ColorDetector()
    det.set_frame(_make_bgr_frame(bgr=(0, 0, 255)))  # 红
    color, conf = det.classify()
    assert color == GridColor.UNKNOWN


@pytest.mark.fast
def test_no_frame_returns_unknown():
    """未设置帧时返回 UNKNOWN + 0 置信度。"""
    det = ColorDetector()
    color, conf = det.classify()
    assert color == GridColor.UNKNOWN
    assert conf == 0.0


@pytest.mark.fast
def test_checkerboard_green_gray_classifies_as_unknown():
    """绿灰交错 → 都不超过阈值，返回 UNKNOWN。

    用 8×8 小棋盘格（20px），确保 ROI 内两种颜色几乎等量。
    使用更小的 ROI（30%）进一步降低不均衡风险。
    """
    width, height = 320, 240
    payload = np.zeros((height, width, 3), dtype=np.uint8)
    for gy in range(12):
        for gx in range(16):
            is_green = (gx + gy) % 2 == 0
            color = (0, 200, 0) if is_green else (230, 230, 230)
            y0, x0 = gy * 20, gx * 20
            payload[y0:y0+20, x0:x0+20] = color
    frame = VideoFrame(0, 0.0, width, height, "bgr24", payload)

    det = ColorDetector(ColorDetectorConfig(
        roi_fraction=0.3,
        color_threshold=0.55,  # 稍微提高阈值，确保等量时不过
    ))
    det.set_frame(frame)
    color, conf = det.classify()
    # 两种颜色都不超过 0.55
    assert color == GridColor.UNKNOWN
    assert 0.4 <= conf <= 0.55


@pytest.mark.fast
def test_checkerboard_mostly_green():
    """大部分为绿色的帧 → GREEN。"""
    frame = _make_checkerboard_frame(
        block_size=80, color_a=(0, 255, 0), color_b=(80, 180, 80)  # 两种绿
    )
    det = ColorDetector()
    det.set_frame(frame)
    color, conf = det.classify()
    assert color == GridColor.GREEN
    assert conf >= 0.5


@pytest.mark.fast
def test_clear_resets_state():
    """clear 后应回到无帧状态。"""
    det = ColorDetector()
    det.set_frame(_make_bgr_frame(bgr=(0, 255, 0)))
    assert det.classify()[0] == GridColor.GREEN
    det.clear()
    assert det.classify()[0] == GridColor.UNKNOWN


# ── 传感器融合 ──────────────────────────────────────

@pytest.mark.fast
def test_classify_at_position_unknown_falls_back_to_expected():
    """视觉 UNKNOWN 时回退到预期颜色。"""
    det = ColorDetector()
    # 不设帧 → UNKNOWN
    color, conf = det.classify_at_position(expected=GridColor.GREEN)
    assert color == GridColor.GREEN  # 回退到预期
    assert conf == 0.30


@pytest.mark.fast
def test_classify_at_position_vision_matches_expectation():
    """视觉与预期一致 → 信任视觉。"""
    det = ColorDetector()
    det.set_frame(_make_bgr_frame(bgr=(0, 255, 0)))  # 绿色
    color, conf = det.classify_at_position(expected=GridColor.GREEN)
    assert color == GridColor.GREEN
    assert conf >= 0.9  # 高置信度


@pytest.mark.fast
def test_classify_at_position_high_conf_vision_overrides():
    """视觉高置信度与预期冲突 → 覆盖预期。"""
    det = ColorDetector()
    det.set_frame(_make_bgr_frame(bgr=(0, 255, 0)))  # 视觉看到绿
    # 但位置预期说应该是灰
    color, conf = det.classify_at_position(
        expected=GridColor.GRAY, confidence_override=0.5
    )
    assert color == GridColor.GREEN  # 视觉覆盖
    assert conf >= 0.9


@pytest.mark.fast
def test_classify_at_position_low_conf_vision_trusts_expectation():
    """视觉低置信度（≈50%）与预期冲突 → 信任预期。"""
    det = ColorDetector()
    # 8×8 小棋盘格 → 两种颜色近等量 → UNKNOWN
    width, height = 320, 240
    payload = np.zeros((height, width, 3), dtype=np.uint8)
    for gy in range(12):
        for gx in range(16):
            is_green = (gx + gy) % 2 == 0
            color = (0, 200, 0) if is_green else (230, 230, 230)
            y0, x0 = gy * 20, gx * 20
            payload[y0:y0+20, x0:x0+20] = color
    frame = VideoFrame(0, 0.0, width, height, "bgr24", payload)
    det.set_frame(frame)
    color, conf = det.classify_at_position(
        expected=GridColor.GRAY,
        confidence_override=0.8,
    )
    assert color == GridColor.GRAY  # 视觉不足以覆盖预期
    assert conf == 0.50


# ── ROI 切割测试 ─────────────────────────────────────

@pytest.mark.fast
def test_roi_excludes_edge_colors():
    """ROI 只取中心，边缘颜色不应影响判定。"""
    # 帧：边缘全红，中心全绿
    width, height = 320, 240
    payload = np.full((height, width, 3), (0, 0, 255), dtype=np.uint8)  # 全红
    # 中心区域覆盖绿色
    cx, cy = width // 2, height // 2
    roi_h, roi_w = int(height * 0.3), int(width * 0.3)
    payload[cy - roi_h // 2: cy + roi_h // 2,
            cx - roi_w // 2: cx + roi_w // 2] = (0, 255, 0)

    frame = VideoFrame(0, 0.0, width, height, "bgr24", payload)
    det = ColorDetector(ColorDetectorConfig(roi_fraction=0.3))
    det.set_frame(frame)
    color, conf = det.classify()
    assert color == GridColor.GREEN  # 只看中心，应该为绿
    assert conf >= 0.8
