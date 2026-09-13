"""下视相机像素射线到机体/世界相对位置的显式变换。"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self) -> None:
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError("相机焦距必须为正")


class DownwardCameraModel:
    """C系为右/下/光轴向前，B系为前/右/上。"""

    def __init__(self, intrinsics: CameraIntrinsics, rotation_body_from_camera=None) -> None:
        self.intrinsics = intrinsics
        if rotation_body_from_camera is None:
            # 正装下视：图像向下对应机体前，图像向右对应机体右，光轴对应机体向下。
            rotation_body_from_camera = (
                (0.0, 1.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.0, 0.0, -1.0),
            )
        matrix = np.asarray(rotation_body_from_camera, dtype=float)
        if matrix.shape != (3, 3):
            raise ValueError("rotation_body_from_camera必须为3x3")
        self.rotation_body_from_camera = matrix

    def body_relative_xy(self, u: float, v: float, relative_height_m: float) -> tuple[float, float]:
        if relative_height_m <= 0:
            raise ValueError("相对高度必须为正")
        k = self.intrinsics
        ray_camera = np.array(((u - k.cx) / k.fx, (v - k.cy) / k.fy, 1.0))
        ray_body = self.rotation_body_from_camera @ ray_camera
        if ray_body[2] >= -1e-6:
            raise ValueError("相机射线未指向平台平面，请检查外参")
        scale = -relative_height_m / ray_body[2]
        point = ray_body * scale
        return float(point[0]), float(point[1])

    @staticmethod
    def world_relative_xy(body_xy: tuple[float, float], yaw_rad: float) -> tuple[float, float]:
        x_body, y_body = body_xy
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        return c * x_body - s * y_body, s * x_body + c * y_body
