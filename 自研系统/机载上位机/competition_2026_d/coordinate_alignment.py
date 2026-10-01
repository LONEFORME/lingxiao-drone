"""双 T265 到统一场地坐标系的二维刚体标定工具。

本模块只做纯数学计算，不访问串口、T265 或飞控。正式控制代码在采用标定
结果前，应先通过独立采集工具验证坐标轴、旋转、平移、尺度和残差。
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
from typing import Mapping, Sequence

import numpy as np


# 题图给出的场地全局坐标：左下角为原点，向右 +X，向上 +Y。
OFFICIAL_FIELD_POINTS_M: dict[str, tuple[float, float]] = {
    "H": (0.75, 0.75),
    "A": (1.50, 2.00),
    "B": (1.50, 3.50),
    "C": (3.00, 3.50),
    "D": (3.00, 2.00),
}


@dataclass(frozen=True)
class RigidTransform2D:
    """从某一 T265/通信坐标系到场地全局坐标系的二维刚体变换。"""

    rotation_deg: float
    translation_x_m: float
    translation_y_m: float
    scale_diagnostic: float
    rms_error_m: float
    max_error_m: float
    mirrored_fit_rms_m: float
    reflection_suspected: bool
    point_count: int

    @property
    def rotation_rad(self) -> float:
        return math.radians(self.rotation_deg)

    def apply(self, xy_m: Sequence[float]) -> tuple[float, float]:
        x_m, y_m = map(float, xy_m)
        cosine = math.cos(self.rotation_rad)
        sine = math.sin(self.rotation_rad)
        return (
            cosine * x_m - sine * y_m + self.translation_x_m,
            sine * x_m + cosine * y_m + self.translation_y_m,
        )

    def to_dict(self) -> dict:
        return asdict(self)


def _as_points(points: Sequence[Sequence[float]], name: str) -> np.ndarray:
    array = np.asarray(points, dtype=float)
    if array.ndim != 2 or array.shape[1] != 2 or array.shape[0] < 2:
        raise ValueError(f"{name}至少需要2个二维点")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name}包含非有限数值")
    centered = array - np.mean(array, axis=0)
    if float(np.sum(centered * centered)) < 1e-8:
        raise ValueError(f"{name}缺少足够的空间跨度")
    return array


def _fit_matrix(
    source_centered: np.ndarray,
    target_centered: np.ndarray,
    *,
    allow_reflection: bool,
) -> np.ndarray:
    covariance = source_centered.T @ target_centered
    left, _singular, right_t = np.linalg.svd(covariance)
    rotation = right_t.T @ left.T
    if not allow_reflection and np.linalg.det(rotation) < 0.0:
        right_t[-1, :] *= -1.0
        rotation = right_t.T @ left.T
    return rotation


def fit_rigid_transform_2d(
    source_points_m: Sequence[Sequence[float]],
    field_points_m: Sequence[Sequence[float]],
) -> RigidTransform2D:
    """拟合 ``field = R @ source + t``，并输出尺度/镜像诊断。

    正式变换始终限制为旋转和平移，不自动吸收尺度或镜像错误。尺度偏离1或
    镜像拟合显著更好时，应修正上游坐标定义，而不是把异常变换写进飞控。
    """

    source = _as_points(source_points_m, "source_points_m")
    target = _as_points(field_points_m, "field_points_m")
    if source.shape != target.shape:
        raise ValueError("source_points_m与field_points_m数量必须一致")

    source_mean = np.mean(source, axis=0)
    target_mean = np.mean(target, axis=0)
    source_centered = source - source_mean
    target_centered = target - target_mean

    rotation = _fit_matrix(
        source_centered,
        target_centered,
        allow_reflection=False,
    )
    translation = target_mean - rotation @ source_mean
    predicted = (rotation @ source.T).T + translation
    errors = np.linalg.norm(predicted - target, axis=1)

    rotated_centered = (rotation @ source_centered.T).T
    source_energy = float(np.sum(rotated_centered * rotated_centered))
    scale = float(
        np.sum(rotated_centered * target_centered) / source_energy
    )

    mirrored = _fit_matrix(
        source_centered,
        target_centered,
        allow_reflection=True,
    )
    mirrored_translation = target_mean - mirrored @ source_mean
    mirrored_predicted = (mirrored @ source.T).T + mirrored_translation
    mirrored_errors = np.linalg.norm(mirrored_predicted - target, axis=1)
    mirrored_rms = float(np.sqrt(np.mean(mirrored_errors * mirrored_errors)))

    rotation_deg = math.degrees(
        math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    )
    rms_error = float(np.sqrt(np.mean(errors * errors)))
    reflection_suspected = bool(
        np.linalg.det(mirrored) < 0.0
        and mirrored_rms + 0.02 < rms_error
        and mirrored_rms < 0.5 * rms_error
    )
    return RigidTransform2D(
        rotation_deg=rotation_deg,
        translation_x_m=float(translation[0]),
        translation_y_m=float(translation[1]),
        scale_diagnostic=scale,
        rms_error_m=rms_error,
        max_error_m=float(np.max(errors)),
        mirrored_fit_rms_m=mirrored_rms,
        reflection_suspected=reflection_suspected,
        point_count=int(source.shape[0]),
    )


def fit_labeled_measurements(
    measurements: Mapping[str, Sequence[float]],
    field_points: Mapping[str, Sequence[float]] = OFFICIAL_FIELD_POINTS_M,
) -> tuple[RigidTransform2D, dict[str, dict[str, float]]]:
    """按共同点名拟合变换，并返回每个点的预测值和误差。"""

    labels = sorted(set(measurements) & set(field_points))
    if len(labels) < 3:
        raise ValueError("至少需要3个不同的已知场地点")
    source = [measurements[label] for label in labels]
    target = [field_points[label] for label in labels]
    transform = fit_rigid_transform_2d(source, target)
    residuals: dict[str, dict[str, float]] = {}
    for label, source_xy, target_xy in zip(labels, source, target):
        predicted = transform.apply(source_xy)
        error = math.hypot(
            predicted[0] - float(target_xy[0]),
            predicted[1] - float(target_xy[1]),
        )
        residuals[label] = {
            "source_x_m": float(source_xy[0]),
            "source_y_m": float(source_xy[1]),
            "expected_x_m": float(target_xy[0]),
            "expected_y_m": float(target_xy[1]),
            "predicted_x_m": predicted[0],
            "predicted_y_m": predicted[1],
            "error_m": error,
        }
    return transform, residuals


__all__ = [
    "OFFICIAL_FIELD_POINTS_M",
    "RigidTransform2D",
    "fit_labeled_measurements",
    "fit_rigid_transform_2d",
]
