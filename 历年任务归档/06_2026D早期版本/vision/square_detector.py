"""
square_detector.py — 朝下摄像头黑色实心方块检测

检测流程：
  1. 灰度化 → 自适应阈值 → 形态学闭运算（去噪、填孔）
  2. 外轮廓提取 → 多边形近似 → 筛选 4 边形
  3. 长宽比 + 面积双重过滤，取最大符合轮廓
  4. area_ratio > max_area_ratio 标记 TOO_CLOSE（方块超出 FOV 边界前兆）

无副作用：detect() 不修改输入帧。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import numpy as np


@dataclass
class DetectionResult:
    found: bool
    cx_px: int = 0          # 方块中心 x（像素，图像坐标系左上为原点）
    cy_px: int = 0          # 方块中心 y
    area_ratio: float = 0.0 # 轮廓面积 / 画面面积
    too_close: bool = False  # area_ratio 超上限，方块过近
    frame_w: int = 0
    frame_h: int = 0


class SquareDetector:
    """
    Parameters
    ----------
    min_area_ratio : float
        轮廓面积占画面比例下限，低于此视为噪声（默认 0.002，约 320×240 中 37×37px）
    max_area_ratio : float
        面积占比上限，超过此值置 too_close=True（默认 0.5）
    aspect_range : tuple[float, float]
        宽高比允许范围（默认 0.65 ~ 1.55，容纳一定透视变形）
    block_size : int
        自适应阈值邻域大小，需为奇数（默认 21）
    c_offset : int
        自适应阈值偏置（默认 10）
    morph_ksize : int
        形态学闭运算核大小（默认 7）
    poly_eps : float
        多边形近似精度系数，乘以周长（默认 0.04）
    """

    def __init__(
        self,
        min_area_ratio: float = 0.002,
        max_area_ratio: float = 0.50,
        aspect_range: tuple[float, float] = (0.65, 1.55),
        block_size: int = 21,
        c_offset: int = 10,
        morph_ksize: int = 7,
        poly_eps: float = 0.04,
    ) -> None:
        self.min_area_ratio = min_area_ratio
        self.max_area_ratio = max_area_ratio
        self.aspect_range = aspect_range
        self.block_size = block_size
        self.c_offset = c_offset
        self._morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (morph_ksize, morph_ksize)
        )
        self.poly_eps = poly_eps

    # ------------------------------------------------------------------ #
    def detect(self, frame: np.ndarray) -> DetectionResult:
        """
        Parameters
        ----------
        frame : np.ndarray
            BGR 或灰度图（H×W×3 或 H×W）

        Returns
        -------
        DetectionResult
        """
        if frame is None or frame.size == 0:
            return DetectionResult(found=False)

        h, w = frame.shape[:2]
        frame_area = float(h * w)

        # 1. 灰度化
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame

        # 2. 自适应阈值（黑色方块 → 反色后为白色轮廓）
        binary = cv2.adaptiveThreshold(
            gray, 255,
            cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY_INV,
            self.block_size,
            self.c_offset,
        )

        # 3. 形态学闭运算（填小空洞、连通断裂边缘）
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self._morph_kernel)

        # 4. 外轮廓
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        best_cnt = None
        best_area = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            ratio = area / frame_area

            if ratio < self.min_area_ratio:
                continue

            # 多边形近似
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, self.poly_eps * peri, True)
            if len(approx) != 4:
                continue

            # 长宽比过滤
            _, _, bw, bh = cv2.boundingRect(approx)
            if bh == 0:
                continue
            aspect = bw / bh
            lo, hi = self.aspect_range
            if not (lo <= aspect <= hi):
                continue

            if area > best_area:
                best_area = area
                best_cnt = approx

        if best_cnt is None:
            return DetectionResult(found=False, frame_w=w, frame_h=h)

        # 矩心
        M = cv2.moments(best_cnt)
        if M["m00"] == 0:
            return DetectionResult(found=False, frame_w=w, frame_h=h)

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        area_ratio = best_area / frame_area

        return DetectionResult(
            found=True,
            cx_px=cx,
            cy_px=cy,
            area_ratio=area_ratio,
            too_close=(area_ratio > self.max_area_ratio),
            frame_w=w,
            frame_h=h,
        )
