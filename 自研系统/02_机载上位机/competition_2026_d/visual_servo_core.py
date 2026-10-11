"""视觉观测到世界速度指令的无硬件副作用核心。"""

from __future__ import annotations

from dataclasses import dataclass

from .control.formation_controller import FormationCommand, FormationController
from .vision.camera_model import DownwardCameraModel
from .vision.platform_observation import FeatureFlag, PlatformObservation
from .vision.platform_tracker import PlatformTracker


@dataclass(frozen=True)
class DroneKinematics:
    x_m: float
    y_m: float
    z_m: float
    vx_m_s: float
    vy_m_s: float
    yaw_rad: float


class VisualServoCore:
    def __init__(
        self,
        camera: DownwardCameraModel,
        tracker: PlatformTracker | None = None,
        controller: FormationController | None = None,
        max_observation_age_s: float = 0.15,
        min_quality: int = 55,
        allow_surrogate: bool = False,
    ) -> None:
        self.camera = camera
        self.tracker = tracker or PlatformTracker()
        self.controller = controller or FormationController()
        self.max_observation_age_s = max_observation_age_s
        self.min_quality = min_quality
        self.allow_surrogate = allow_surrogate
        self._last_key: tuple[int, int] | None = None

    def tick(
        self,
        now: float,
        drone: DroneKinematics,
        observation: PlatformObservation | None,
        relative_height_m: float,
        desired_offset_xy: tuple[float, float] = (0.0, 0.0),
    ) -> FormationCommand:
        flags = FeatureFlag(observation.flags) if observation is not None else FeatureFlag(0)
        if observation is not None and not (flags & FeatureFlag.PARTIAL) and observation.usable(
            now,
            self.max_observation_age_s,
            self.min_quality,
            allow_surrogate=self.allow_surrogate,
        ):
            key = (observation.stream_id, observation.seq)
            if key != self._last_key:
                body_xy = self.camera.body_relative_xy(
                    observation.cx, observation.cy, relative_height_m
                )
                world_xy = self.camera.world_relative_xy(body_xy, drone.yaw_rad)
                self.tracker.update(
                    drone.x_m + world_xy[0],
                    drone.y_m + world_xy[1],
                    observation.received_monotonic,
                    observation.quality,
                )
                self._last_key = key
        estimate = self.tracker.predict(now)
        return self.controller.command(
            estimate,
            (drone.x_m, drone.y_m),
            (drone.vx_m_s, drone.vy_m_s),
            now,
            desired_offset_xy,
        )
