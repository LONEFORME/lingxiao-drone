"""小车正向安装 T265 的底盘中心位姿处理。

坐标约定：

* ``+X``：小车启动时的车头方向；
* ``+Y``：小车启动时的左侧；
* ``+Z``：向上；
* Yaw 左转为正。

本模块与无人机现有 ``t265.py`` 独立。它先把 T265 原生 Pose 转换到
小车初始世界坐标，再补偿 T265 相对底盘旋转中心的杆臂，最后才进行滤波。
"""

from __future__ import annotations

from dataclasses import dataclass
import logging
import math
import threading
import time
from typing import Optional, Sequence

import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:  # 电脑离线测试不要求安装旧版 RealSense SDK
    rs = None


LOGGER = logging.getLogger(__name__)

# T265: +X 向右、+Y 向上、+Z 向后（镜头前方为 -Z）
# 小车: +X 向前、+Y 向左、+Z 向上
SENSOR_TO_CAR = np.array(
    [
        [0.0, 0.0, -1.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=float,
)


def wrap_pi(angle_rad: float) -> float:
    """把角度限制到 ``[-pi, pi)``。"""

    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_xyzw_to_matrix(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    """把 ``(x, y, z, w)`` 四元数转换为 3x3 旋转矩阵。"""

    quaternion = np.asarray(quaternion_xyzw, dtype=float)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion_xyzw 必须是4个有限数值")
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-9:
        raise ValueError("四元数模长过小")
    x, y, z, w = quaternion / norm
    return np.array(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=float,
    )


@dataclass(frozen=True)
class CarT265Config:
    """小车 T265 安装与数据质量配置。"""

    mount_forward_m: float = 0.2525
    mount_left_m: float = -0.0070
    mount_up_m: float = 0.0
    min_confidence: int = 2
    calibration_duration_s: float = 2.0
    calibration_min_samples: int = 30
    position_filter_alpha: float = 0.20
    velocity_filter_alpha: float = 0.20
    max_pose_age_s: float = 0.25
    max_position_jump_m: float = 0.30
    velocity_reset_gap_s: float = 0.50
    frame_timeout_ms: int = 1000

    def __post_init__(self) -> None:
        if self.min_confidence not in (0, 1, 2, 3):
            raise ValueError("min_confidence 必须在0到3之间")
        if self.calibration_duration_s < 0.0:
            raise ValueError("calibration_duration_s 不能为负数")
        if self.calibration_min_samples < 1:
            raise ValueError("calibration_min_samples 必须至少为1")
        for name in ("position_filter_alpha", "velocity_filter_alpha"):
            value = float(getattr(self, name))
            if not 0.0 < value <= 1.0:
                raise ValueError(f"{name} 必须在(0, 1]范围内")
        if self.max_pose_age_s <= 0.0:
            raise ValueError("max_pose_age_s 必须大于0")
        if self.max_position_jump_m <= 0.0:
            raise ValueError("max_position_jump_m 必须大于0")
        if self.velocity_reset_gap_s <= 0.0:
            raise ValueError("velocity_reset_gap_s 必须大于0")
        if self.frame_timeout_ms < 1:
            raise ValueError("frame_timeout_ms 必须至少为1")

    @property
    def lever_arm_body_m(self) -> np.ndarray:
        """从底盘旋转中心指向 T265 的机体系杆臂。"""

        return np.array(
            [self.mount_forward_m, self.mount_left_m, self.mount_up_m],
            dtype=float,
        )


@dataclass(frozen=True)
class CarT265Status:
    ready: bool
    calibrating: bool
    running: bool
    confidence: int
    pose_age_s: float
    healthy: bool
    accepted_frames: int
    rejected_low_confidence: int
    rejected_jump: int
    last_error: Optional[str]


class CarT265Pose:
    """读取并输出经过杆臂补偿的底盘旋转中心位姿。"""

    def __init__(self, config: Optional[CarT265Config] = None) -> None:
        self.config = config or CarT265Config()
        self._lock = threading.RLock()
        self._pipeline = None
        self._thread: Optional[threading.Thread] = None
        self._running = False

        self._ready = False
        self._calibrating = True
        self._calibration_started_at: Optional[float] = None
        self._calibration_positions: list[np.ndarray] = []
        self._calibration_quaternions: list[np.ndarray] = []

        self._origin_sensor_position = np.zeros(3, dtype=float)
        self._origin_car_rotation = np.eye(3, dtype=float)
        self._origin_yaw_rad = 0.0

        self._raw_center_position = np.zeros(3, dtype=float)
        self._center_position = np.zeros(3, dtype=float)
        self._center_velocity = np.zeros(3, dtype=float)
        self._heading_rad = 0.0
        self._last_update_timestamp: Optional[float] = None
        self._last_valid_timestamp: Optional[float] = None
        self._last_frame_timestamp: Optional[float] = None
        self._confidence = 0

        self._accepted_frames = 0
        self._rejected_low_confidence = 0
        self._rejected_jump = 0
        self._last_error: Optional[str] = None

    def begin_calibration(self) -> None:
        """清除旧原点，开始收集静止且置信度达标的标定样本。"""

        with self._lock:
            self._ready = False
            self._calibrating = True
            # 标定计时从首个有效样本开始，低置信度等待时间不能计入静止平均。
            self._calibration_started_at = None
            self._calibration_positions.clear()
            self._calibration_quaternions.clear()
            self._raw_center_position.fill(0.0)
            self._center_position.fill(0.0)
            self._center_velocity.fill(0.0)
            self._heading_rad = 0.0
            self._last_update_timestamp = None
            self._last_valid_timestamp = None

    def update_pose(
        self,
        translation_xyz: Sequence[float],
        quaternion_xyzw: Sequence[float],
        confidence: int,
        timestamp: Optional[float] = None,
    ) -> bool:
        """处理一帧 T265 Pose；返回该帧是否成为有效输出。

        ``translation_xyz`` 与 ``quaternion_xyzw`` 均使用 T265 原生坐标。
        低置信度、非递增时间戳和异常位置跳变不会覆盖最后的有效输出。
        """

        now = time.monotonic() if timestamp is None else float(timestamp)
        translation = np.asarray(translation_xyz, dtype=float)
        quaternion = np.asarray(quaternion_xyzw, dtype=float)
        if translation.shape != (3,) or not np.all(np.isfinite(translation)):
            raise ValueError("translation_xyz 必须是3个有限数值")
        if not math.isfinite(now):
            raise ValueError("timestamp 必须是有限数值")
        rotation_sensor = quaternion_xyzw_to_matrix(quaternion)
        quaternion = quaternion / np.linalg.norm(quaternion)

        sensor_position_car_world = SENSOR_TO_CAR @ translation
        car_rotation = SENSOR_TO_CAR @ rotation_sensor @ SENSOR_TO_CAR.T

        with self._lock:
            if (
                self._last_frame_timestamp is not None
                and now <= self._last_frame_timestamp
            ):
                self._last_error = "non_monotonic_timestamp"
                return False
            self._last_frame_timestamp = now
            self._confidence = int(confidence)

            if self._confidence < self.config.min_confidence:
                self._rejected_low_confidence += 1
                return False

            if self._calibrating:
                return self._consume_calibration_sample(
                    sensor_position_car_world,
                    quaternion,
                    now,
                )

            center_position = self._compensate_center(
                sensor_position_car_world,
                car_rotation,
            )
            if (
                self._last_update_timestamp is not None
                and float(
                    np.linalg.norm(center_position - self._raw_center_position)
                )
                > self.config.max_position_jump_m
            ):
                self._rejected_jump += 1
                self._last_error = "position_jump"
                return False

            self._accept_center_pose(center_position, car_rotation, now)
            return True

    def _consume_calibration_sample(
        self,
        sensor_position: np.ndarray,
        quaternion: np.ndarray,
        timestamp: float,
    ) -> bool:
        if self._calibration_started_at is None:
            self._calibration_started_at = timestamp
        if (
            self._calibration_quaternions
            and float(
                np.dot(self._calibration_quaternions[0], quaternion)
            )
            < 0.0
        ):
            quaternion = -quaternion
        self._calibration_positions.append(sensor_position.copy())
        self._calibration_quaternions.append(quaternion.copy())

        elapsed = timestamp - self._calibration_started_at
        if (
            elapsed < self.config.calibration_duration_s
            or len(self._calibration_positions)
            < self.config.calibration_min_samples
        ):
            return False

        average_quaternion = np.mean(
            np.asarray(self._calibration_quaternions),
            axis=0,
        )
        norm = float(np.linalg.norm(average_quaternion))
        if norm < 1e-9:
            self._last_error = "calibration_quaternion_invalid"
            return False
        average_quaternion /= norm

        self._origin_sensor_position = np.mean(
            np.asarray(self._calibration_positions),
            axis=0,
        )
        origin_sensor_rotation = quaternion_xyzw_to_matrix(average_quaternion)
        self._origin_car_rotation = (
            SENSOR_TO_CAR
            @ origin_sensor_rotation
            @ SENSOR_TO_CAR.T
        )
        self._origin_yaw_rad = math.atan2(
            self._origin_car_rotation[1, 0],
            self._origin_car_rotation[0, 0],
        )
        self._raw_center_position.fill(0.0)
        self._center_position.fill(0.0)
        self._center_velocity.fill(0.0)
        self._heading_rad = 0.0
        self._last_update_timestamp = timestamp
        self._last_valid_timestamp = timestamp
        self._accepted_frames += 1
        self._ready = True
        self._calibrating = False
        self._last_error = None
        return True

    def _compensate_center(
        self,
        sensor_position: np.ndarray,
        car_rotation: np.ndarray,
    ) -> np.ndarray:
        sensor_delta = sensor_position - self._origin_sensor_position
        lever_delta = (
            car_rotation - self._origin_car_rotation
        ) @ self.config.lever_arm_body_m
        return sensor_delta - lever_delta

    def _accept_center_pose(
        self,
        center_position: np.ndarray,
        car_rotation: np.ndarray,
        timestamp: float,
    ) -> None:
        previous_raw = self._raw_center_position.copy()
        previous_timestamp = self._last_update_timestamp
        alpha_position = self.config.position_filter_alpha
        alpha_velocity = self.config.velocity_filter_alpha

        self._raw_center_position = center_position.copy()
        self._center_position = (
            alpha_position * center_position
            + (1.0 - alpha_position) * self._center_position
        )

        if previous_timestamp is None:
            raw_velocity = np.zeros(3, dtype=float)
        else:
            dt = timestamp - previous_timestamp
            if 0.0 < dt <= self.config.velocity_reset_gap_s:
                raw_velocity = (center_position - previous_raw) / dt
            else:
                raw_velocity = np.zeros(3, dtype=float)
        self._center_velocity = (
            alpha_velocity * raw_velocity
            + (1.0 - alpha_velocity) * self._center_velocity
        )

        yaw_rad = math.atan2(car_rotation[1, 0], car_rotation[0, 0])
        self._heading_rad = wrap_pi(yaw_rad - self._origin_yaw_rad)
        self._last_update_timestamp = timestamp
        self._last_valid_timestamp = timestamp
        self._accepted_frames += 1
        self._last_error = None

    def get_center_position(self) -> np.ndarray:
        with self._lock:
            return self._center_position.copy()

    def get_raw_center_position(self) -> np.ndarray:
        """返回杆臂补偿后、低通滤波前的底盘中心位置。"""

        with self._lock:
            return self._raw_center_position.copy()

    def get_center_velocity(self) -> np.ndarray:
        """返回中心位置差分速度；生产控制优先使用轮编码器速度。"""

        with self._lock:
            return self._center_velocity.copy()

    def get_heading_rad(self) -> float:
        with self._lock:
            return float(self._heading_rad)

    def get_heading_cdeg(self) -> int:
        """返回协议常用的 0.01 度航向值。"""

        return int(round(math.degrees(self.get_heading_rad()) * 100.0))

    def get_tracking_confidence(self) -> int:
        with self._lock:
            return int(self._confidence)

    def get_pose_age_s(self, now: Optional[float] = None) -> float:
        current = time.monotonic() if now is None else float(now)
        with self._lock:
            timestamp = self._last_valid_timestamp
        if timestamp is None:
            return math.inf
        return max(0.0, current - timestamp)

    def is_ready(self) -> bool:
        with self._lock:
            return bool(self._ready)

    def is_healthy(self, now: Optional[float] = None) -> bool:
        with self._lock:
            ready = self._ready
            confidence = self._confidence
        return (
            ready
            and confidence >= self.config.min_confidence
            and self.get_pose_age_s(now) <= self.config.max_pose_age_s
        )

    def get_status(self, now: Optional[float] = None) -> CarT265Status:
        age = self.get_pose_age_s(now)
        with self._lock:
            ready = self._ready
            confidence = self._confidence
            return CarT265Status(
                ready=ready,
                calibrating=self._calibrating,
                running=self._running,
                confidence=confidence,
                pose_age_s=age,
                healthy=(
                    ready
                    and confidence >= self.config.min_confidence
                    and age <= self.config.max_pose_age_s
                ),
                accepted_frames=self._accepted_frames,
                rejected_low_confidence=self._rejected_low_confidence,
                rejected_jump=self._rejected_jump,
                last_error=self._last_error,
            )

    def start(self) -> None:
        """启动真实 T265 Pose 采集线程并进入静止标定。"""

        if rs is None:
            raise RuntimeError(
                "未安装支持T265的pyrealsense2；T265通常需要librealsense 2.50.x"
            )
        with self._lock:
            if self._running:
                return

        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.pose, rs.format.any, 200)
        pipeline.start(config)
        with self._lock:
            self._pipeline = pipeline
            self._running = True
            self._last_error = None
        self.begin_calibration()
        self._thread = threading.Thread(
            target=self._acquisition_loop,
            name="car-t265-pose",
            daemon=True,
        )
        self._thread.start()

    def _acquisition_loop(self) -> None:
        while True:
            with self._lock:
                if not self._running:
                    return
                pipeline = self._pipeline
            try:
                frames = pipeline.wait_for_frames(
                    self.config.frame_timeout_ms
                )
                pose_frame = frames.get_pose_frame()
                if not pose_frame:
                    continue
                pose = pose_frame.get_pose_data()
                self.update_pose(
                    (
                        pose.translation.x,
                        pose.translation.y,
                        pose.translation.z,
                    ),
                    (
                        pose.rotation.x,
                        pose.rotation.y,
                        pose.rotation.z,
                        pose.rotation.w,
                    ),
                    int(getattr(pose, "tracker_confidence", 0)),
                    time.monotonic(),
                )
            except Exception as exc:  # 硬件线程不能无提示退出
                with self._lock:
                    if not self._running:
                        return
                    self._last_error = str(exc)
                LOGGER.warning("小车T265采集异常: %s", exc)
                time.sleep(0.05)

    def wait_until_ready(self, timeout_s: float = 8.0) -> bool:
        deadline = time.monotonic() + float(timeout_s)
        while time.monotonic() < deadline:
            if self.is_ready():
                return True
            with self._lock:
                if not self._running:
                    return False
            time.sleep(0.02)
        return self.is_ready()

    def stop(self) -> None:
        with self._lock:
            self._running = False
            pipeline = self._pipeline
            self._pipeline = None
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass
        thread = self._thread
        if (
            thread is not None
            and thread.is_alive()
            and thread is not threading.current_thread()
        ):
            thread.join(timeout=2.0)
        self._thread = None

    def __enter__(self) -> "CarT265Pose":
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.stop()


__all__ = [
    "CarT265Config",
    "CarT265Pose",
    "CarT265Status",
    "SENSOR_TO_CAR",
    "quaternion_xyzw_to_matrix",
    "wrap_pi",
]
