"""颜色识别模块 — 识别画面中心 ROI 的主色调（绿/灰）。

策略：
  1. 取画面中心 15~20% 区域作为 ROI（避免看到多个网格）
  2. HSV 阈值分割绿色和灰色像素
  3. 主色调投票 + 置信度门槛
  4. 融合 T265 位置预期做传感器级兜底
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

import numpy as np

from Lcode.video_source import VideoFrame


class GridColor(Enum):
    GREEN = auto()    # 播撒区（绿色）
    GRAY = auto()     # 非播撒区（灰色）
    UNKNOWN = auto()  # 无法确定


@dataclass(frozen=True)
class ColorDetectorConfig:
    # ── HSV 绿色阈值 ──
    green_h_min: int = 35
    green_h_max: int = 85
    green_s_min: int = 30
    green_s_max: int = 255
    green_v_min: int = 30
    green_v_max: int = 255

    # ── HSV 灰色阈值（低饱和度 = 灰色） ──
    gray_s_max: int = 35      # 饱和度上限
    gray_v_min: int = 160     # 亮度下界（排除深色）

    # ── ROI 参数 ──
    roi_fraction: float = 0.35  # 取画面中心的比例（宽高各取 35%）
                                # 在 1.5m 高度 ≈ 30cm 地面范围
                                # 小于网格 50cm，可容忍 T265 ±15cm 偏差

    # ── 置信度 ──
    color_threshold: float = 0.50   # 某颜色占比超过此值 → 判定为该颜色
    unknown_threshold: float = 0.30  # 低于此值 → UNKNOWN


class ColorDetector:
    """从视频帧中识别地面颜色。

    Args:
        config: 颜色阈值配置，可现场微调
    """

    def __init__(self, config: Optional[ColorDetectorConfig] = None):
        self.config = config or ColorDetectorConfig()
        self._last_hsv: Optional[np.ndarray] = None
        self._frame_info: dict = {}

    # ── 输入 ──────────────────────────────────────────────

    def set_frame(self, frame: VideoFrame) -> None:
        """输入一帧画面，提取中心 ROI 的 HSV 数据。"""
        self._frame_info = {
            "width": frame.width,
            "height": frame.height,
            "pixel_format": frame.pixel_format,
        }
        hsv = _frame_to_hsv(frame)
        if hsv is not None:
            self._last_hsv = self._crop_roi(hsv)

    def clear(self) -> None:
        """清空缓存的帧数据。"""
        self._last_hsv = None
        self._frame_info = {}

    # ── 判定 ──────────────────────────────────────────────

    def classify(self) -> tuple[GridColor, float]:
        """基于当前帧判定中心区域颜色。

        Returns:
            (GridColor, 置信度 0~1)
        """
        if self._last_hsv is None:
            return (GridColor.UNKNOWN, 0.0)

        green_ratio = _mask_ratio(self._last_hsv, self._green_mask)
        gray_ratio = _mask_ratio(self._last_hsv, self._gray_mask)

        if green_ratio >= self.config.color_threshold:
            return (GridColor.GREEN, green_ratio)
        if gray_ratio >= self.config.color_threshold:
            return (GridColor.GRAY, gray_ratio)

        best = max(green_ratio, gray_ratio)
        if best >= self.config.unknown_threshold:
            # 有倾向但不足以确定，回退到预期
            return (GridColor.UNKNOWN, best)

        return (GridColor.UNKNOWN, best)

    def classify_at_position(
        self,
        expected: GridColor,
        confidence_override: float = 0.6,
    ) -> tuple[GridColor, float]:
        """融合视觉判定与位置预期。

        规则：
          - 视觉 UNKNOWN → 信任预期（置信度 0.3）
          - 视觉与预期一致 → 信任视觉
          - 视觉与预期冲突且置信度高 → 信任视觉（覆盖预期）
          - 视觉与预期冲突且置信度中低 → 信任预期（兜底）

        Args:
            expected: 当前位置的预期颜色（来自航线规划）
            confidence_override: 视觉覆盖预期的置信度门槛

        Returns:
            (最终颜色, 最终置信度)
        """
        visual, conf = self.classify()

        if visual is GridColor.UNKNOWN:
            return (expected, 0.30)

        if visual == expected:
            return (visual, conf)

        # 冲突：视觉 ≠ 预期
        if conf >= confidence_override:
            return (visual, conf)
        else:
            return (expected, 0.50)

    # ── 内部 ──────────────────────────────────────────────

    def _crop_roi(self, hsv: np.ndarray) -> np.ndarray:
        h, w = hsv.shape[:2]
        f = self.config.roi_fraction
        x0 = int(w * (1 - f) / 2)
        y0 = int(h * (1 - f) / 2)
        x1 = int(w * (1 + f) / 2)
        y1 = int(h * (1 + f) / 2)
        return hsv[y0:y1, x0:x1]

    def _green_mask(self, hsv: np.ndarray) -> np.ndarray:
        c = self.config
        lower = np.array([c.green_h_min, c.green_s_min, c.green_v_min])
        upper = np.array([c.green_h_max, c.green_s_max, c.green_v_max])
        return cv2_inRange(hsv, lower, upper)

    def _gray_mask(self, hsv: np.ndarray) -> np.ndarray:
        c = self.config
        lower = np.array([0, 0, c.gray_v_min])
        upper = np.array([180, c.gray_s_max, 255])
        return cv2_inRange(hsv, lower, upper)


# ── 工具 ──────────────────────────────────────────────


def _frame_to_hsv(frame: VideoFrame) -> Optional[np.ndarray]:
    """将 VideoFrame 转为 HSV numpy 数组。

    支持：
      - payload 为 numpy BGR 数组
      - payload 为 JPEG bytes（需要 OpenCV 解码）
    """
    payload = frame.payload
    if isinstance(payload, np.ndarray):
        if payload.ndim == 3 and payload.shape[2] == 3:
            return cv2_cvtColor(payload, cv2_COLOR_BGR2HSV)
        return None
    if isinstance(payload, (bytes, bytearray)):
        try:
            bgr = cv2_imdecode(payload)
            if bgr is not None:
                return cv2_cvtColor(bgr, cv2_COLOR_BGR2HSV)
        except Exception:
            pass
        return None
    return None


def _mask_ratio(hsv: np.ndarray, mask_fn) -> float:
    """计算 mask 像素占总像素的比例。

    cv2.inRange 返回 uint8 掩码（0/255），需先归一化到 0/1。
    """
    total = hsv.shape[0] * hsv.shape[1]
    if total == 0:
        return 0.0
    mask = mask_fn(hsv)
    return float(mask.astype(np.float64).sum() / total / 255.0)


# ── OpenCV 懒加载 ────────────────────────────────────
# 仅在实际需要时才导入 cv2，方便纯 numpy 环境测试

_cv2 = None


def _lazy_cv2():
    global _cv2
    if _cv2 is None:
        import cv2 as _cv2
    return _cv2


def cv2_inRange(hsv, lower, upper):
    return _lazy_cv2().inRange(hsv, lower, upper)


def cv2_cvtColor(src, code):
    return _lazy_cv2().cvtColor(src, code)


def cv2_imdecode(buf):
    return _lazy_cv2().imdecode(np.frombuffer(buf, np.uint8), _lazy_cv2().IMREAD_COLOR)


cv2_COLOR_BGR2HSV = 40  # cv2.COLOR_BGR2HSV 的值
