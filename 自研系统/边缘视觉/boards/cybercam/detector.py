"""
detector.py — 黑色实心方块检测（运行在 CyberCAM 板端）

目标：1920×1080 画面中检测深色实心方形区域，
返回中心偏移量 (dx, dy)。

依赖：OpenCV（CyberCAM 核桃派自带）
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class DetResult:
    found: bool
    dx: int = 0       # 中心 X 偏移（像素，右为正）
    dy: int = 0       # 中心 Y 偏移（像素，下为正）
    area_ratio: float = 0.0  # 轮廓面积 / 画面面积


class SquareDetector:
    """
    灰度 → 二值化 → 形态学 → 轮廓 → 4边形筛选 → 矩心偏移。

    参数对应 1920×1080 画面，min_area 降低到 0.001（约 50×50px）
    以覆盖目标从低空到高空的尺度范围。
    """

    def __init__(self, img_w: int = 1920, img_h: int = 1080) -> None:
        self.W = img_w
        self.H = img_h
        self._cx0 = img_w // 2   # 画面中心 X (960)
        self._cy0 = img_h // 2   # 画面中心 Y (540)
        self._kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))

    def detect(self, frame: np.ndarray) -> DetResult:
        """检测黑色方块，返回中心偏移。"""
        if frame is None or frame.size == 0:
            return DetResult(found=False)

        h, w = frame.shape[:2]
        # 只处理需要的分辨率（协议约定 1920×1080，但兼容性检查）
        if w != self.W or h != self.H:
            frame = cv2.resize(frame, (self.W, self.H))
        frame_area = float(self.W * self.H)

        # 灰度 → 自适应阈值（反色，黑色变白色）
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        binary = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
            cv2.THRESH_BINARY_INV, 21, 10,
        )
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self._kernel)

        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE,
        )

        best_cnt = None
        best_area = 0.0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            ar = area / frame_area
            if ar < 0.001 or ar > 0.50:   # 太小忽略 / 太大（几乎占满画面）忽略
                continue
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            if len(approx) != 4:
                continue
            x, y, bw, bh = cv2.boundingRect(approx)
            aspect = bw / bh if bh > 0 else 0
            if not (0.65 <= aspect <= 1.55):
                continue
            if area > best_area:
                best_area = area
                best_cnt = approx

        if best_cnt is None:
            return DetResult(found=False)

        M = cv2.moments(best_cnt)
        if M["m00"] == 0:
            return DetResult(found=False)

        cx = int(round(M["m10"] / M["m00"]))
        cy = int(round(M["m01"] / M["m00"]))

        return DetResult(
            found=True,
            dx=cx - self._cx0,
            dy=cy - self._cy0,
            area_ratio=best_area / frame_area,
        )
