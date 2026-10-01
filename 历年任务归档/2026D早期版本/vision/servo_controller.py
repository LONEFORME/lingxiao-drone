"""
servo_controller.py — tick-based IBVS 视觉伺服控制器（v4）

设计原则：
  - 无副作用：tick() 不调用任何飞控接口，只返回速度修正量
  - 无阻塞：每次调用在毫秒内完成，适合在30ms 主循环中直接调用
  - 无线程：状态机完全内置，由调用方（Mission_GPT）驱动

输入源抽象：
  tick() 接受 Detection（像素偏移），不关心检测来自何处。
  - CyberCAM 模式：VideoSource → SquareDetector → Detection
  - USB 摄像头调试模式（桌面测试）：
      vision/utils.py 中的 FrameDetectorAdapter 提供 Detection

调用方（Mission_GPT._visual_servo_tick）每30ms：
  tick = self._vs_ctrl.tick(detection, pos[2])
  if tick.done/failed → self.state = "LAND"
  else → self.set_speed(tick.vx_cm_s, tick.vy_cm_s, yaw_cmd, current_z)
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

from .square_detector import DetectionResult, SquareDetector


# ═══════════════════════════════════════════════════════════════════════ #
# Detection — 统一的检测结果数据类
# ═══════════════════════════════════════════════════════════════════════ #

@dataclass
class Detection:
    """视觉检测结果（像素偏移 + 画面尺寸）。"""
    found: bool
    dx_px: int = 0       # 目标中心相对画面中心的 X 偏移（右为正）
    dy_px: int = 0       # 目标中心相对画面中心的 Y 偏移（下为正）
    frame_w: int = 1920  # 画面宽度（用于缩放换算）
    frame_h: int = 1080  # 画面高度


# ═══════════════════════════════════════════════════════════════════════ #
# FrameDetectorAdapter — 帧→Detection 转换（桌面调试/USB 摄像头）
# ═══════════════════════════════════════════════════════════════════════ #

class FrameDetectorAdapter:
    """
    桌面调试辅助：用 OpenCV 处理帧 → 输出 Detection 对象。
    这样 servo_controller.py 可以在没有 CyberCAM 的情况下测试。
    """

    def __init__(self, img_w: int = 320, img_h: int = 240) -> None:
        self._detector = SquareDetector()
        self._w = img_w
        self._h = img_h

    def detect(self, frame: Optional) -> Optional[Detection]:
        if frame is None or frame.size == 0:
            return None
        res: DetectionResult = self._detector.detect(frame)
        if not res.found:
            return Detection(found=False, frame_w=res.frame_w, frame_h=res.frame_h)
        cx = res.cx_px
        cy = res.cy_px
        return Detection(
            found=True,
            dx_px=cx - res.frame_w // 2,
            dy_px=cy - res.frame_h // 2,
            frame_w=res.frame_w,
            frame_h=res.frame_h,
        )


# ═══════════════════════════════════════════════════════════════════════ #
# Config
# ═══════════════════════════════════════════════════════════════════════ #

@dataclass
class ServoConfig:
    # 相机参数（由标定结果更新）
    focal_length_px: float = 1100.0   # 1920×1080 下典型值，需 calib.py 测量

    # 高度阈值
    alt_stop_m: float = 0.30    # 低于此高度返回 done=True（交 Mission_GPT 盲降）

    # 收敛判据
    centering_threshold_m: float = 0.05    # 位置误差 < 5cm 视为对中
    centering_consec_frames: int = 5       # 连续满足的帧数

    # 速度限制
    max_correction_cm_s: float = 20.0     # 单轴最大修正速度（cm/s）

    # 比例增益（误差1m → 速度 kp×100 cm/s）
    kp: float = 1.0

    # 超时
    search_timeout_s: float = 5.0
    centering_timeout_s: float = 15.0


# ═══════════════════════════════════════════════════════════════════════ #
# Output dataclass
# ═══════════════════════════════════════════════════════════════════════ #

@dataclass
class ServoTick:
    vx_cm_s: float = 0.0    # 前后速度修正（cm/s），+ 为前进
    vy_cm_s: float = 0.0    # 左右速度修正（cm/s），+ 为右移
    state: str = "SEARCHING"
    done: bool = False
    failed: bool = False
    reason: str = ""


# ═══════════════════════════════════════════════════════════════════════ #
# Controller
# ═══════════════════════════════════════════════════════════════════════ #

class _State(Enum):
    SEARCHING = auto()
    CENTERING = auto()


class VisualServoController:
    """
    tick-based IBVS 控制器。接收 Detection（来自 CyberCAM 或 FrameDetectorAdapter
    ），输出速度修正量。

    用法（Mission_GPT._visual_servo_tick 中）：

        tick = self._vs_ctrl.tick(detection, pos[2])
        if tick.done or tick.failed:
            self.state = "LAND"
        else:
            self.set_speed(tick.vx_cm_s, tick.vy_cm_s, yaw_cmd, current_z)
    """

    def __init__(self, config: Optional[ServoConfig] = None) -> None:
        self._cfg = config or ServoConfig()
        self.reset()

    def reset(self) -> None:
        """重置状态机。每次进入 VISUAL_SERVO 状态前调用。"""
        self._state = _State.SEARCHING
        self._state_start = time.monotonic()
        self._consec = 0

    # ── 核心 tick ────────────────────────────────────────────────── #

    def tick(
        self,
        detection: Optional[Detection],
        altitude_m: float,
    ) -> ServoTick:
        """
        由 Mission_GPT.loop() 每 30ms 调用一次。

        Parameters
        ----------
        detection  : CyberCAM 检测结果（或 FrameDetectorAdapter 生成）
        altitude_m : 当前高度（米），来自 Mission_GPT loop 的 pos[2]
        """
        cfg = self._cfg
        now = time.monotonic()
        elapsed = now - self._state_start

        # ── 低于盲降阈值 ──────────────────────────────────────────── #
        if altitude_m < cfg.alt_stop_m:
            return ServoTick(
                done=True,
                state=self._state.name,
                reason="alt_below_stop",
            )

        found = detection is not None and detection.found

        # ── SEARCHING ──────────────────────────────────────────────── #
        if self._state == _State.SEARCHING:
            if elapsed > cfg.search_timeout_s:
                return ServoTick(
                    failed=True,
                    state="SEARCHING",
                    reason="search_timeout",
                )
            if found:
                self._state = _State.CENTERING
                self._state_start = now
                self._consec = 0
            return ServoTick(state="SEARCHING")

        # ── CENTERING ──────────────────────────────────────────────── #
        if elapsed > cfg.centering_timeout_s:
            return ServoTick(
                failed=True,
                state="CENTERING",
                reason="centering_timeout",
            )

        if not found:
            self._consec = 0
            return ServoTick(state="CENTERING")

        # 计算位置误差 & 速度修正
        vx, vy, err_m = self._compute(detection, altitude_m, cfg)

        if err_m < cfg.centering_threshold_m:
            self._consec += 1
        else:
            self._consec = 0

        if self._consec >= cfg.centering_consec_frames:
            return ServoTick(
                done=True, state="CENTERING", reason="centered",
            )

        return ServoTick(vx_cm_s=vx, vy_cm_s=vy, state="CENTERING")

    # ── 计算 ──────────────────────────────────────────────────────── #

    def _compute(
        self,
        det: Detection,
        altitude_m: float,
        cfg: ServoConfig,
    ):
        """
        像素偏移 → 位置误差（米）→ 速度修正（cm/s）。

        坐标映射（标准朝下相机，CyberCAM 坐标系）：
          dx > 0 目标在右侧 → 需要向右飞 → vy > 0
          dy > 0 目标在下侧 → 需要向后飞 → vx < 0（注意：dy>0=下方）
          但在图像坐标系中，dy>0=画面下方=飞机前方→需要向前
        """
        # 防御：使用 detection 的画面尺寸，若无则用配置默认
        w = det.frame_w if det.frame_w > 0 else 1920
        h = det.frame_h if det.frame_h > 0 else 1080

        # 像素偏移 → 米（针孔相机模型）
        scale = altitude_m / cfg.focal_length_px
        err_x_m = float(det.dx_px) * scale   # 飞机左右方向（米）
        err_y_m = float(det.dy_px) * scale   # 飞机前后方向（米）
        err_m = (err_x_m ** 2 + err_y_m ** 2) ** 0.5

        # 米 → cm/s（比例控制，限幅）
        clamp = cfg.max_correction_cm_s
        # 目标在右 → vy > 0（向右）
        vy = float(max(min(err_x_m * 100.0 * cfg.kp, clamp), -clamp))
        # 目标在画面下方（dy>0）→ 目标在飞机前方 → 向前飞 vx > 0
        vx = float(max(min(err_y_m * 100.0 * cfg.kp, clamp), -clamp))

        return vx, vy, err_m
