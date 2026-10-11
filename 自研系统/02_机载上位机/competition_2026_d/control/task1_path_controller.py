"""任务一固定地图路径控制器。

水平速度只由固定路径、小车标称速度和 T265 位置闭环生成。视觉观测不会进入
本模块，避免视觉与航点控制争夺飞控 XY 速度写入权。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class PathSample:
    s_m: float
    point_xy_m: tuple[float, float]
    tangent_xy: tuple[float, float]
    remaining_m: float


@dataclass(frozen=True)
class PathCommand:
    vx_m_s: float
    vy_m_s: float
    progress_m: float
    remaining_m: float
    completed: bool
    target_xy_m: tuple[float, float]


class PolylinePath:
    def __init__(self, points: Iterable[tuple[float, float]]) -> None:
        self.points = tuple((float(x), float(y)) for x, y in points)
        if len(self.points) < 2:
            raise ValueError("路径至少需要两个点")
        self._segments: list[tuple[float, float, float, float, float]] = []
        self._cumulative = [0.0]
        for start, end in zip(self.points, self.points[1:]):
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length = math.hypot(dx, dy)
            if length <= 1e-6:
                raise ValueError("路径包含重合的相邻点")
            self._segments.append((start[0], start[1], dx, dy, length))
            self._cumulative.append(self._cumulative[-1] + length)
        self.length_m = self._cumulative[-1]

    def sample(self, s_m: float) -> PathSample:
        s = min(max(0.0, float(s_m)), self.length_m)
        index = len(self._segments) - 1
        for candidate in range(len(self._segments)):
            if s <= self._cumulative[candidate + 1]:
                index = candidate
                break
        x0, y0, dx, dy, length = self._segments[index]
        local = min(max(s - self._cumulative[index], 0.0), length)
        ratio = local / length
        tangent = (dx / length, dy / length)
        return PathSample(
            s_m=s,
            point_xy_m=(x0 + ratio * dx, y0 + ratio * dy),
            tangent_xy=tangent,
            remaining_m=max(0.0, self.length_m - s),
        )

    def project(
        self,
        position_xy_m: tuple[float, float],
        *,
        minimum_s_m: float = 0.0,
    ) -> PathSample:
        px, py = position_xy_m
        minimum_s = min(max(0.0, minimum_s_m), self.length_m)
        best: tuple[float, float] | None = None
        for index, (x0, y0, dx, dy, length) in enumerate(self._segments):
            segment_start_s = self._cumulative[index]
            segment_end_s = self._cumulative[index + 1]
            if segment_end_s + 1e-9 < minimum_s:
                continue
            ratio = ((px - x0) * dx + (py - y0) * dy) / (length * length)
            ratio = min(max(ratio, 0.0), 1.0)
            candidate_s = segment_start_s + ratio * length
            if candidate_s < minimum_s:
                candidate_s = minimum_s
            sample = self.sample(candidate_s)
            distance = math.hypot(
                px - sample.point_xy_m[0], py - sample.point_xy_m[1]
            )
            if best is None or distance < best[0]:
                best = (distance, candidate_s)
        return self.sample(minimum_s if best is None else best[1])


@dataclass(frozen=True)
class Task1PathFollowerConfig:
    lookahead_m: float = 0.20
    cross_track_kp: float = 0.80
    max_correction_m_s: float = 0.08
    max_speed_m_s: float = 0.20
    max_accel_m_s2: float = 0.30
    completion_radius_m: float = 0.14


class Task1PathFollower:
    def __init__(
        self,
        path: PolylinePath,
        config: Task1PathFollowerConfig | None = None,
    ) -> None:
        self.path = path
        self.config = config or Task1PathFollowerConfig()
        self.progress_m = 0.0
        self._last_time: float | None = None
        self._last_velocity = (0.0, 0.0)

    def reset(
        self,
        *,
        progress_m: float = 0.0,
        timestamp: float | None = None,
        velocity_xy_m_s: tuple[float, float] = (0.0, 0.0),
    ) -> None:
        self.progress_m = min(max(0.0, progress_m), self.path.length_m)
        self._last_time = timestamp
        self._last_velocity = _limit_norm(
            float(velocity_xy_m_s[0]),
            float(velocity_xy_m_s[1]),
            self.config.max_speed_m_s,
        )

    def command(
        self,
        position_xy_m: tuple[float, float],
        *,
        nominal_speed_m_s: float,
        timestamp: float,
    ) -> PathCommand:
        nearest = self.path.project(
            position_xy_m,
            minimum_s_m=max(0.0, self.progress_m - 0.03),
        )
        self.progress_m = max(self.progress_m, nearest.s_m)
        lookahead = self.path.sample(
            min(self.path.length_m, self.progress_m + self.config.lookahead_m)
        )

        correction_x = self.config.cross_track_kp * (
            nearest.point_xy_m[0] - position_xy_m[0]
        )
        correction_y = self.config.cross_track_kp * (
            nearest.point_xy_m[1] - position_xy_m[1]
        )
        correction_x, correction_y = _limit_norm(
            correction_x, correction_y, self.config.max_correction_m_s
        )
        speed = max(0.0, float(nominal_speed_m_s))
        target_vx = lookahead.tangent_xy[0] * speed + correction_x
        target_vy = lookahead.tangent_xy[1] * speed + correction_y
        target_vx, target_vy = _limit_norm(
            target_vx, target_vy, self.config.max_speed_m_s
        )
        vx, vy = self._limit_acceleration(target_vx, target_vy, timestamp)

        endpoint = self.path.points[-1]
        endpoint_distance = math.hypot(
            position_xy_m[0] - endpoint[0], position_xy_m[1] - endpoint[1]
        )
        completed = (
            self.progress_m >= self.path.length_m - self.config.completion_radius_m
            and endpoint_distance <= self.config.completion_radius_m
        )
        return PathCommand(
            vx_m_s=vx,
            vy_m_s=vy,
            progress_m=self.progress_m,
            remaining_m=max(0.0, self.path.length_m - self.progress_m),
            completed=completed,
            target_xy_m=lookahead.point_xy_m,
        )

    def _limit_acceleration(
        self, target_vx: float, target_vy: float, timestamp: float
    ) -> tuple[float, float]:
        if self._last_time is None or timestamp <= self._last_time:
            self._last_time = timestamp
            self._last_velocity = (target_vx, target_vy)
            return self._last_velocity
        dt = min(timestamp - self._last_time, 0.20)
        delta_x = target_vx - self._last_velocity[0]
        delta_y = target_vy - self._last_velocity[1]
        delta_x, delta_y = _limit_norm(
            delta_x, delta_y, self.config.max_accel_m_s2 * dt
        )
        self._last_velocity = (
            self._last_velocity[0] + delta_x,
            self._last_velocity[1] + delta_y,
        )
        self._last_time = timestamp
        return self._last_velocity


def _limit_norm(x: float, y: float, limit: float) -> tuple[float, float]:
    norm = math.hypot(x, y)
    if norm <= limit or norm <= 1e-9:
        return x, y
    scale = limit / norm
    return x * scale, y * scale
