"""任务二静止平台带桨复飞测试。

流程：使用正式任务已经验证的 Mission_GPT 控制链完成
起飞 -> H 点定点升至目标高度 -> 悬停 -> H 点定点下降 -> 平台接触，
保持解锁且旋翼转动 5 秒后直接复升，再完成最终下降和锁桨。

中途不产生第二次解锁或第二次一键起飞，T265也不重启、不重新校零。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

BASIC_DIR = Path(__file__).resolve().parents[1] / "basic"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

import Lcode.Lprotocol  # noqa: E402
from Lcode.global_variable import lock, sp_side  # noqa: E402
from Mission_GPT import (  # noqa: E402
    TAKEOFF_LIFTOFF_CM,
    T265_CONFIDENCE_MIN,
    laser_height_valid,
    mission,
)
from t265 import t265_class  # noqa: E402

from .task2_retakeoff_bench import (  # noqa: E402
    BenchFailure,
    _format_snapshot,
    _position_tuple,
    _read_fc_snapshot,
    _set_fc_command,
    _state_matches,
    _wait_for_fc_state,
    _wait_for_t265,
)


CONFIRM_TEXT = "STATIC_PLATFORM_PROPS_READY"


@dataclass(frozen=True)
class StaticFlightConfig:
    height_m: float = 1.00
    airborne_hold_s: float = 2.0
    deck_hold_s: float = 5.0
    final_closed_loop_height_m: float = 0.15
    cycle_timeout_s: float = 55.0
    lock_timeout_s: float = 12.0
    t265_timeout_s: float = 8.0
    max_xy_excursion_m: float = 0.35
    max_fc_age_s: float = 0.25
    max_pwm_age_s: float = 4.0
    confirm_count: int = 5


class StaticRetakeoffMission(mission):
    """复用正式任务控制器，仅提供两周期状态复位和 H 点航点。"""

    def __init__(
        self,
        re_fc,
        se_fc,
        realsense_obj,
        serial_fc_ref,
        config: StaticFlightConfig,
        anchor_xy_m: tuple[float, float],
    ) -> None:
        super().__init__(
            re_fc,
            se_fc,
            realsense_obj,
            serial_fc_ref,
            interactive_preflight=False,
            preflight_warning_completed=True,
        )
        self.config = config
        self.anchor_xy_m = (
            float(anchor_xy_m[0]),
            float(anchor_xy_m[1]),
        )
        self.cycle_number = 0
        self.t265_ok = True
        self._install_static_waypoints()

    def _install_static_waypoints(self) -> None:
        anchor_x, anchor_y = self.anchor_xy_m
        self.targets = [
            [anchor_x, anchor_y, self.config.height_m],
            [anchor_x, anchor_y, self.config.final_closed_loop_height_m],
        ]

    def _waypoint_hold_s(self) -> float:
        if self.target_index == 0:
            return self.config.airborne_hold_s
        return 0.0

    def prepare_cycle(self, cycle_number: int) -> None:
        """复位任务状态，但保留同一个 T265 实例及其校准原点。"""
        self.cycle_number = int(cycle_number)
        self._install_static_waypoints()
        self.target_index = 0
        self.last_target_index = -1
        self._arrival_window.clear()
        self._vel_window.clear()
        self.arrival_start_time = 0.0
        self.arrival_confirmed_time = None
        self._cruise_arrival_count = 0
        self._active_segment_distance_m = 0.0
        self._ramp_z_cm = 0.0
        self.x_pid.reset()
        self.y_pid.reset()
        self.heading_hold.reset_for_new_mission()
        self._last_heading_fault_logged = None
        self.emergency_stop = False
        for name in (
            "_descend_start",
            "_descend_confirm",
            "_descend_gaveup_logged",
            "_hover_wait_start",
        ):
            if hasattr(self, name):
                delattr(self, name)
        with lock:
            self.se_fc[2] = 0
            self.se_fc[3] = sp_side
            self.se_fc[4] = sp_side
            self.se_fc[5] = 0
            self.se_fc[6] = sp_side
            self.se_fc[7] = 0
        self.state = "TAKEOFF"

    def prepare_unlocked_reascent(self, current_height_m: float) -> None:
        """保持task_sta=1和电机解锁，仅复位航点状态后直接复升。"""
        self.cycle_number = 2
        self._install_static_waypoints()
        self.target_index = 0
        self.last_target_index = -1
        self._arrival_window.clear()
        self._vel_window.clear()
        self.arrival_start_time = 0.0
        self.arrival_confirmed_time = None
        self._cruise_arrival_count = 0
        self._active_segment_distance_m = 0.0
        self._ramp_z_cm = max(0.0, float(current_height_m) * 100.0)
        self.x_pid.reset()
        self.y_pid.reset()
        for name in (
            "_descend_start",
            "_descend_confirm",
            "_descend_gaveup_logged",
            "_hover_wait_start",
        ):
            if hasattr(self, name):
                delattr(self, name)
        with lock:
            self.se_fc[2] = 1
            self.se_fc[3] = sp_side
            self.se_fc[4] = sp_side
            self.se_fc[5] = int(round(self._ramp_z_cm))
            self.se_fc[7] = 0
        self.state = "NAVIGATE"

    def begin_firmware_takeoff(self) -> None:
        """触发固件起飞，但不阻塞等待飞到35cm后才启用位置闭环。"""
        confidence = int(self.realsense.get_tracking_confidence())
        if confidence < T265_CONFIDENCE_MIN:
            raise BenchFailure(f"起飞前T265置信度不足({confidence})")
        yaw = float(self.realsense.get_orientation()[2])
        self._heading_status = self.heading_hold.arm(yaw, time.time())
        with lock:
            self.se_fc[2] = 1
            self.se_fc[3] = sp_side
            self.se_fc[4] = sp_side
            self.se_fc[5] = int(TAKEOFF_LIFTOFF_CM)
            self.se_fc[6] = (
                int(self._heading_status.command_dps) + sp_side
            )
            self.se_fc[7] = 0


def _wait_for_firmware_takeoff_handoff(
    mission_obj: StaticRetakeoffMission,
    timeout_s: float = 8.0,
) -> None:
    """固件接受起飞后立即交给成熟NAVIGATE位置PID，不等待35cm。"""
    deadline = time.monotonic() + float(timeout_s)
    last_print = 0.0
    while time.monotonic() < deadline:
        snapshot = _read_fc_snapshot(
            mission_obj.re_fc, mission_obj.serial_fc_ref
        )
        confidence = int(mission_obj.realsense.get_tracking_confidence())
        if confidence < T265_CONFIDENCE_MIN:
            raise BenchFailure(f"起飞交接期间T265置信度不足({confidence})")
        yaw = float(mission_obj.realsense.get_orientation()[2])
        mission_obj._heading_status = mission_obj._update_heading_hold(
            yaw, confidence
        )
        with lock:
            mission_obj.se_fc[6] = (
                int(mission_obj._heading_status.command_dps) + sp_side
            )

        if snapshot.mission_stage == 5:
            mission_obj._ramp_z_cm = float(TAKEOFF_LIFTOFF_CM)
            mission_obj.state = "NAVIGATE"
            print(
                "[HANDOFF] 固件起飞指令已接受，立即启用成熟H点位置PID；"
                "不等待飞到35cm"
            )
            return

        now = time.monotonic()
        if now - last_print >= 0.5:
            print(
                "[WAIT HANDOFF] "
                f"stage={snapshot.mission_stage}, {_format_snapshot(snapshot)}"
            )
            last_print = now
        time.sleep(0.03)
    raise BenchFailure("固件起飞阶段交接超时")


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--propellers-installed", action="store_true")
    parser.add_argument("--static-platform", action="store_true")
    parser.add_argument("--height", type=float, default=1.00)
    return parser


def _config_from_args(args) -> StaticFlightConfig:
    if not args.propellers_installed or not args.static_platform:
        raise SystemExit(
            "拒绝运行：必须同时添加 --propellers-installed --static-platform"
        )
    if not 0.40 <= float(args.height) <= 1.00:
        raise SystemExit(
            "--height 必须在 0.40~1.00m 之间；成熟起飞逻辑会先升至约0.35m"
        )
    return StaticFlightConfig(height_m=float(args.height))


def _position_with_laser(mission_obj: StaticRetakeoffMission):
    position = list(mission_obj.realsense.get_position())
    yaw = float(mission_obj.realsense.get_orientation()[2])
    with lock:
        laser_height_m = float(mission_obj.serial_fc_ref._last_laser_height_cm)
    if laser_height_valid(laser_height_m):
        position[2] = laser_height_m
    return position, yaw


def _check_navigation_guards(
    mission_obj: StaticRetakeoffMission,
    position,
) -> None:
    confidence = int(mission_obj.realsense.get_tracking_confidence())
    pose_age_s = float(mission_obj.realsense.get_pose_age_s())
    if confidence < 2:
        raise BenchFailure(f"T265置信度不足({confidence})")
    if pose_age_s > 0.30:
        raise BenchFailure(f"T265 Pose超时({pose_age_s:.2f}s)")
    dx = float(position[0]) - mission_obj.anchor_xy_m[0]
    dy = float(position[1]) - mission_obj.anchor_xy_m[1]
    if math.hypot(dx, dy) > mission_obj.config.max_xy_excursion_m:
        raise BenchFailure(
            f"水平偏移超限: relative_xy=({dx:+.3f},{dy:+.3f})m"
        )


def _run_until_platform_contact(
    mission_obj: StaticRetakeoffMission,
    cycle_number: int,
) -> None:
    deadline = time.monotonic() + mission_obj.config.cycle_timeout_s
    last_print = 0.0
    while time.monotonic() < deadline:
        if mission_obj.state == "LAND":
            print(
                f"[CONTACT] 第{cycle_number}次已到平台近地门槛；"
                "保持解锁，不调用LAND/FC_Lock"
            )
            return
        if mission_obj.state == "HOVER_WAIT":
            raise BenchFailure(
                f"第{cycle_number}次进入HOVER_WAIT，等待人工接管"
            )

        position, yaw = _position_with_laser(mission_obj)
        if mission_obj.state == "NAVIGATE":
            _check_navigation_guards(mission_obj, position)
            mission_obj.navigate(position, yaw)
        elif mission_obj.state == "DESCEND":
            mission_obj.descend(position)
        else:
            raise BenchFailure(
                f"第{cycle_number}次出现未知飞行状态: {mission_obj.state}"
            )

        now = time.monotonic()
        if now - last_print >= 0.5:
            with lock:
                command_x = int(mission_obj.se_fc[3]) - sp_side
                command_y = int(mission_obj.se_fc[4]) - sp_side
                command_height = int(mission_obj.se_fc[5])
            relative_x = float(position[0]) - mission_obj.anchor_xy_m[0]
            relative_y = float(position[1]) - mission_obj.anchor_xy_m[1]
            print(
                f"[FLIGHT {cycle_number}] state={mission_obj.state}, "
                f"target={mission_obj.target_index}/{len(mission_obj.targets)}, "
                f"h={position[2]:.3f}m, "
                f"relative_xy=({relative_x:+.3f},{relative_y:+.3f})m, "
                f"cmd_xy=({command_x:+d},{command_y:+d})cm/s, "
                f"cmd_h={command_height}cm"
            )
            last_print = now
        time.sleep(0.03)

    raise BenchFailure(f"第{cycle_number}次到达平台超时")


def _hold_unlocked_on_platform(
    mission_obj: StaticRetakeoffMission,
    read_fc,
) -> float:
    """平台接触期间保持解锁和通信，禁止发送降落/锁桨命令。"""
    with lock:
        mission_obj.se_fc[2] = 1
        mission_obj.se_fc[3] = sp_side
        mission_obj.se_fc[4] = sp_side
        mission_obj.se_fc[5] = 0
        mission_obj.se_fc[7] = 0
    deadline = time.monotonic() + mission_obj.config.deck_hold_s
    last_height_m = 0.0
    while time.monotonic() < deadline:
        snapshot = read_fc()
        if not _state_matches(
            snapshot,
            unlocked=True,
            max_fc_age_s=mission_obj.config.max_fc_age_s,
            max_pwm_age_s=mission_obj.config.max_pwm_age_s,
        ):
            raise BenchFailure(
                "平台停留期间意外锁桨或反馈异常: "
                + _format_snapshot(snapshot)
            )
        position, _yaw = _position_with_laser(mission_obj)
        last_height_m = max(0.0, float(position[2]))
        time.sleep(0.03)
    print(
        f"[PASS] 平台保持解锁、旋翼转动 "
        f"{mission_obj.config.deck_hold_s:.1f}s"
    )
    return last_height_m


def _wait_for_manual_lock(read_fc, se_fc) -> None:
    """空中异常后绝不由异常收尾路径锁桨，等待遥控手安全降落。"""
    snapshot = read_fc()
    if snapshot.unlock_sta == 1:
        with lock:
            current_height_cm = max(8, int(se_fc[5]))
        _set_fc_command(
            se_fc,
            task_sta=1,
            next_task_sign=0,
            height_cm=current_height_cm,
        )
    else:
        _set_fc_command(se_fc, task_sta=0, next_task_sign=0, height_cm=0)
    print("[MANUAL] 请立即遥控接管、安全降落并锁桨；程序保持串口通信。")
    last_print = 0.0
    while True:
        snapshot = read_fc()
        if _state_matches(
            snapshot,
            unlocked=False,
            max_fc_age_s=0.25,
            max_pwm_age_s=4.0,
        ):
            print("[PASS] 已确认人工锁桨")
            return
        now = time.monotonic()
        if now - last_print >= 0.5:
            print(f"[WAIT MANUAL LOCK] {_format_snapshot(snapshot)}")
            last_print = now
        time.sleep(0.05)


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    config = _config_from_args(args)

    print("=" * 70)
    print("任务二静止平台带桨复飞测试（复用正式 Mission_GPT 控制链）")
    print(
        f"两次目标高度={config.height_m:.2f}m，悬停={config.airborne_hold_s:.1f}s，"
        f"中途平台解锁停留={config.deck_hold_s:.1f}s"
    )
    print("必须使用水平静止平台，遥控手全程准备接管。")
    print("=" * 70)
    confirmation = input(f"请输入 {CONFIRM_TEXT} 继续: ").strip()
    if confirmation != CONFIRM_TEXT:
        print("确认文本不匹配，未打开飞控串口。")
        return 2

    port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
    re_fc = [0] * 14
    se_fc = [
        170,
        2,
        0,
        sp_side,
        sp_side,
        0,
        sp_side,
        0,
        sp_side,
        0,
        255,
    ]
    realsense = t265_class()
    serial_fc = None
    send_started = False
    safely_locked = False

    try:
        if getattr(realsense, "use_simulation", False):
            raise BenchFailure("pyrealsense2/T265不可用，禁止带桨起飞")
        serial_fc = Lcode.Lprotocol.Serial_fc(port, 460800)
        serial_fc.listen_start(re_fc)
        serial_fc.send_start(se_fc, realsense, vel_freq=100, cmd_freq=50)
        send_started = True
        read_fc = lambda: _read_fc_snapshot(re_fc, serial_fc)

        _wait_for_fc_state(
            read_fc,
            unlocked=False,
            label="初始锁桨状态",
            timeout_s=config.lock_timeout_s,
            max_fc_age_s=config.max_fc_age_s,
            max_pwm_age_s=config.max_pwm_age_s,
            confirm_count=config.confirm_count,
        )
        safely_locked = True

        if not realsense.start():
            raise BenchFailure("T265启动失败")
        _wait_for_t265(realsense, config.t265_timeout_s)
        realsense.autoset()
        anchor = _position_tuple(realsense)
        print(f"[PASS] T265只启动/校零一次，H点={anchor}")

        mission_obj = StaticRetakeoffMission(
            re_fc,
            se_fc,
            realsense,
            serial_fc,
            config,
            (anchor[0], anchor[1]),
        )

        print("红灯安全倒计时5秒；任何人不得靠近螺旋桨。")
        for remaining in range(5, 0, -1):
            print(f"[COUNTDOWN] {remaining}")
            time.sleep(1.0)

        safely_locked = False
        mission_obj.prepare_cycle(1)
        print("[CYCLE 1] 首次起飞")
        mission_obj.begin_firmware_takeoff()
        _wait_for_firmware_takeoff_handoff(mission_obj)
        _run_until_platform_contact(mission_obj, 1)

        before_second = _position_tuple(realsense)
        contact_height_m = _hold_unlocked_on_platform(mission_obj, read_fc)
        mission_obj.prepare_unlocked_reascent(contact_height_m)
        print("[CYCLE 2] 不解锁、不发送第二次一键起飞，直接复升")
        _run_until_platform_contact(mission_obj, 2)

        print("[FINAL] 最终降落，执行锁桨确认")
        mission_obj.land()
        if mission_obj.state != "END":
            raise BenchFailure(
                f"最终锁桨未完成，当前状态={mission_obj.state}"
            )
        safely_locked = True
        after_second = _position_tuple(realsense)
        print(
            "[PASS] T265中途未重启/未再次校零: "
            f"before={before_second}, after={after_second}"
        )
        print(
            "[RESULT] 首次起飞、平台不锁桨停留、直接复升、"
            "最终降落锁桨均成功"
        )
        return 0
    except KeyboardInterrupt:
        print("\n[ABORT] 收到中断")
        return 130
    except BenchFailure as exc:
        print(f"[FAIL] {exc}")
        return 1
    finally:
        if serial_fc is not None and send_started and not safely_locked:
            try:
                _wait_for_manual_lock(
                    lambda: _read_fc_snapshot(re_fc, serial_fc),
                    se_fc,
                )
                safely_locked = True
            except Exception as exc:
                print(f"[CRITICAL] 人工接管等待异常: {exc}")
        if serial_fc is not None:
            if safely_locked:
                _set_fc_command(
                    se_fc, task_sta=0, next_task_sign=0, height_cm=0
                )
                time.sleep(0.1)
            serial_fc.send_end()
            serial_fc.close()
        if realsense.is_running():
            realsense.stop()


if __name__ == "__main__":
    raise SystemExit(main())
