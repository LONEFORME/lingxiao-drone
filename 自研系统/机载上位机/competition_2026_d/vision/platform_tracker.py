"""平台世界位置/速度的四状态恒速度线性卡尔曼跟踪器。"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PlatformEstimate:
    x_m: float
    y_m: float
    vx_m_s: float
    vy_m_s: float
    timestamp: float
    uncertainty_m: float
    predicted: bool


@dataclass(frozen=True)
class TrackerConfig:
    process_accel_std_m_s2: float = 0.80
    measurement_std_best_m: float = 0.025
    measurement_std_worst_m: float = 0.18
    max_speed_m_s: float = 1.5
    max_innovation_m: float = 0.8
    max_predict_s: float = 0.15
    initial_uncertainty_m: float = 0.25
    initial_velocity_uncertainty_m_s: float = 0.8


class PlatformTracker:
    """状态为[x, y, vx, vy]，所有时间均为调用方单调时钟。"""

    def __init__(self, config: TrackerConfig | None = None) -> None:
        self.config = config or TrackerConfig()
        self._x: np.ndarray | None = None
        self._p: np.ndarray | None = None
        self._timestamp: float | None = None
        self.rejected = 0

    @property
    def initialized(self) -> bool:
        return self._x is not None

    def reset(self) -> None:
        self._x = None
        self._p = None
        self._timestamp = None
        self.rejected = 0

    def update(
        self, x_m: float, y_m: float, timestamp: float, quality: int = 100
    ) -> PlatformEstimate | None:
        if not all(math.isfinite(value) for value in (x_m, y_m, timestamp)):
            self.rejected += 1
            return None
        if self._x is None:
            cfg = self.config
            self._x = np.array([x_m, y_m, 0.0, 0.0], dtype=float)
            self._p = np.diag([
                cfg.initial_uncertainty_m ** 2,
                cfg.initial_uncertainty_m ** 2,
                cfg.initial_velocity_uncertainty_m_s ** 2,
                cfg.initial_velocity_uncertainty_m_s ** 2,
            ])
            self._timestamp = float(timestamp)
            return self._estimate(timestamp, False)
        if timestamp <= self._timestamp:
            self.rejected += 1
            return None

        predicted_x, predicted_p = self._predict_state(timestamp)
        innovation = np.array([x_m - predicted_x[0], y_m - predicted_x[1]])
        if float(np.linalg.norm(innovation)) > self.config.max_innovation_m:
            self.rejected += 1
            return None

        quality_01 = max(0.0, min(1.0, float(quality) / 100.0))
        cfg = self.config
        measurement_std = (
            cfg.measurement_std_worst_m
            - quality_01 * (cfg.measurement_std_worst_m - cfg.measurement_std_best_m)
        )
        h = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]])
        r = np.eye(2) * measurement_std ** 2
        s = h @ predicted_p @ h.T + r
        k = predicted_p @ h.T @ np.linalg.inv(s)
        updated_x = predicted_x + k @ innovation
        identity = np.eye(4)
        # Joseph形式保持有限精度下协方差的对称和半正定。
        updated_p = (identity - k @ h) @ predicted_p @ (identity - k @ h).T + k @ r @ k.T

        speed = math.hypot(float(updated_x[2]), float(updated_x[3]))
        if speed > cfg.max_speed_m_s:
            scale = cfg.max_speed_m_s / speed
            updated_x[2] *= scale
            updated_x[3] *= scale
        self._x = updated_x
        self._p = 0.5 * (updated_p + updated_p.T)
        self._timestamp = float(timestamp)
        return self._estimate(timestamp, False)

    def predict(self, timestamp: float) -> PlatformEstimate | None:
        if self._x is None or not math.isfinite(timestamp) or timestamp < self._timestamp:
            return None
        dt = timestamp - self._timestamp
        if dt > self.config.max_predict_s:
            return None
        predicted_x, predicted_p = self._predict_state(timestamp)
        return self._estimate(timestamp, dt > 0.0, predicted_x, predicted_p)

    def _predict_state(self, timestamp: float) -> tuple[np.ndarray, np.ndarray]:
        dt = max(0.0, float(timestamp) - float(self._timestamp))
        f = np.array([
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        accel_variance = self.config.process_accel_std_m_s2 ** 2
        q_axis = np.array([
            [0.25 * dt ** 4, 0.5 * dt ** 3],
            [0.5 * dt ** 3, dt ** 2],
        ]) * accel_variance
        q = np.zeros((4, 4))
        q[np.ix_([0, 2], [0, 2])] = q_axis
        q[np.ix_([1, 3], [1, 3])] = q_axis
        return f @ self._x, f @ self._p @ f.T + q

    def _estimate(
        self,
        timestamp: float,
        predicted: bool,
        state: np.ndarray | None = None,
        covariance: np.ndarray | None = None,
    ) -> PlatformEstimate:
        state = self._x if state is None else state
        covariance = self._p if covariance is None else covariance
        uncertainty = math.sqrt(max(0.0, float(max(covariance[0, 0], covariance[1, 1]))))
        return PlatformEstimate(
            float(state[0]), float(state[1]), float(state[2]), float(state[3]),
            float(timestamp), uncertainty, bool(predicted),
        )
