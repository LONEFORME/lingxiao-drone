"""任务二拆桨复飞台架测试。

本程序只验证同一进程内的两次解锁：

    首次一键起飞指令 -> 锁桨 -> 保持5秒 -> 飞控任务指令复位
    -> 第二次一键起飞指令 -> 再次锁桨

T265 在整个过程中只启动一次、只执行一次 autoset()，中途不重启。

必须拆掉四个螺旋桨后运行：

    cd /home/sunrise/Desktop/FJJ
    python3 -u -m competition_2026_d.task2_retakeoff_bench \
      --propellers-removed

这不是飞行程序，不会进入任务二路径。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

BASIC_DIR = Path(__file__).resolve().parents[1] / "basic"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

import Lcode.Lprotocol  # noqa: E402
from Lcode.global_variable import (  # noqa: E402
    fc_last_rx_monotonic,
    lock,
    sp_side,
)
from t265 import t265_class  # noqa: E402


CONFIRM_TEXT = "PROPELLERS_REMOVED"
MOTOR_BITS_MASK = 0x0F


class BenchFailure(RuntimeError):
    """拆桨复飞测试未通过。"""


@dataclass(frozen=True)
class FcSnapshot:
    unlock_sta: int | None
    mission_stage: int | None
    motor_pwm_mask: int | None
    fc_age_s: float
    pwm_age_s: float

    @property
    def motor_bits(self) -> int | None:
        if self.motor_pwm_mask is None:
            return None
        return int(self.motor_pwm_mask) & MOTOR_BITS_MASK


def _set_fc_command(
    se_fc: list[int],
    *,
    task_sta: int,
    next_task_sign: int,
    height_cm: int = 0,
) -> None:
    """原子更新 AA02 指令帧的业务字段。"""
    with lock:
        se_fc[2] = int(task_sta)
        se_fc[3] = sp_side
        se_fc[4] = sp_side
        se_fc[5] = int(height_cm)
        se_fc[6] = sp_side
        se_fc[7] = int(next_task_sign)


def _read_fc_snapshot(re_fc, serial_fc) -> FcSnapshot:
    now_mono = time.monotonic()
    now_wall = time.time()
    with lock:
        unlock_sta = int(re_fc[5]) if len(re_fc) > 5 else None
        mission_stage = int(re_fc[0]) if re_fc else None
        debug = dict(getattr(serial_fc, "debug_data", {}) or {})
        last_fc = float(fc_last_rx_monotonic.value)
    pwm_mask = debug.get("motor_pwm_mask")
    pwm_time = debug.get("motor_pwm_mask_t")
    return FcSnapshot(
        unlock_sta=unlock_sta,
        mission_stage=mission_stage,
        motor_pwm_mask=None if pwm_mask is None else int(pwm_mask),
        fc_age_s=math.inf if last_fc <= 0.0 else max(0.0, now_mono - last_fc),
        pwm_age_s=(
            math.inf
            if pwm_time is None
            else max(0.0, now_wall - float(pwm_time))
        ),
    )


def _state_matches(
    snapshot: FcSnapshot,
    *,
    unlocked: bool,
    max_fc_age_s: float,
    max_pwm_age_s: float,
) -> bool:
    """锁桨/解锁必须同时满足解锁位和新鲜PWM反馈。"""
    if snapshot.fc_age_s > max_fc_age_s:
        return False
    if snapshot.pwm_age_s > max_pwm_age_s or snapshot.motor_bits is None:
        return False
    if unlocked:
        return snapshot.unlock_sta == 1 and snapshot.motor_bits != 0
    return snapshot.unlock_sta == 0 and snapshot.motor_bits == 0


def _format_snapshot(snapshot: FcSnapshot) -> str:
    return (
        f"unlock_sta={snapshot.unlock_sta}, "
        f"motor_pwm_mask={snapshot.motor_pwm_mask}, "
        f"motor_bits={snapshot.motor_bits}, "
        f"mission_stage={snapshot.mission_stage}, "
        f"fc_age={snapshot.fc_age_s:.2f}s, "
        f"pwm_age={snapshot.pwm_age_s:.2f}s"
    )


def _wait_for_fc_state(
    read_snapshot: Callable[[], FcSnapshot],
    *,
    unlocked: bool,
    label: str,
    timeout_s: float,
    max_fc_age_s: float,
    max_pwm_age_s: float,
    confirm_count: int,
) -> FcSnapshot:
    deadline = time.monotonic() + timeout_s
    matched = 0
    last_print = 0.0
    latest = read_snapshot()
    while time.monotonic() < deadline:
        latest = read_snapshot()
        if _state_matches(
            latest,
            unlocked=unlocked,
            max_fc_age_s=max_fc_age_s,
            max_pwm_age_s=max_pwm_age_s,
        ):
            matched += 1
            if matched >= confirm_count:
                print(f"[PASS] {label}: {_format_snapshot(latest)}")
                return latest
        else:
            matched = 0

        now = time.monotonic()
        if now - last_print >= 0.5:
            print(f"[WAIT] {label}: {_format_snapshot(latest)}")
            last_print = now
        time.sleep(0.03)

    raise BenchFailure(f"{label}超时: {_format_snapshot(latest)}")


def _wait_for_t265(realsense, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if (
            realsense.get_pose_frame_count() >= 10
            and realsense.get_tracking_confidence() >= 2
            and realsense.get_pose_age_s() <= 0.2
        ):
            return
        time.sleep(0.05)
    raise BenchFailure(
        "T265追踪未在限时内稳定，不发送解锁指令"
    )


def _position_tuple(realsense) -> tuple[float, float, float]:
    position = realsense.get_position()
    return tuple(float(value) for value in position[:3])


def _request_lock_and_confirm(
    se_fc,
    read_snapshot,
    *,
    label: str,
    timeout_s: float,
    max_fc_age_s: float,
    max_pwm_age_s: float,
    confirm_count: int,
) -> FcSnapshot:
    _set_fc_command(
        se_fc,
        task_sta=0,
        next_task_sign=101,
        height_cm=0,
    )
    return _wait_for_fc_state(
        read_snapshot,
        unlocked=False,
        label=label,
        timeout_s=timeout_s,
        max_fc_age_s=max_fc_age_s,
        max_pwm_age_s=max_pwm_age_s,
        confirm_count=confirm_count,
    )


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--propellers-removed",
        action="store_true",
        help="必填安全门禁：确认四个螺旋桨已拆除",
    )
    parser.add_argument("--takeoff-height-cm", type=int, default=30)
    parser.add_argument("--locked-hold-s", type=float, default=5.0)
    parser.add_argument("--motor-observe-s", type=float, default=2.0)
    parser.add_argument("--unlock-timeout-s", type=float, default=8.0)
    parser.add_argument("--lock-timeout-s", type=float, default=12.0)
    parser.add_argument("--t265-timeout-s", type=float, default=8.0)
    parser.add_argument("--max-fc-age-s", type=float, default=0.25)
    # 板端调试扩展帧实测约每2.5秒更新一次，预留4秒容差；
    # 解锁/锁桨仍必须等到一帧新鲜PWM后才会确认通过。
    parser.add_argument("--max-pwm-age-s", type=float, default=4.0)
    parser.add_argument("--confirm-count", type=int, default=5)
    return parser


def _validate_args(args) -> None:
    if not args.propellers_removed:
        raise SystemExit(
            "拒绝运行：必须拆掉四个螺旋桨，并添加 "
            "--propellers-removed"
        )
    if not 20 <= args.takeoff_height_cm <= 50:
        raise SystemExit("--takeoff-height-cm 必须在20~50cm之间")
    for name in (
        "locked_hold_s",
        "motor_observe_s",
        "unlock_timeout_s",
        "lock_timeout_s",
        "t265_timeout_s",
        "max_fc_age_s",
        "max_pwm_age_s",
    ):
        if float(getattr(args, name)) <= 0.0:
            raise SystemExit(f"--{name.replace('_', '-')} 必须大于0")
    if args.confirm_count < 1:
        raise SystemExit("--confirm-count 必须大于等于1")


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    _validate_args(args)

    print("=" * 62)
    print("任务二拆桨复飞测试：过程中电机会两次转动")
    print("确认飞行器已固定、四个螺旋桨均已拆除、周围无杂物。")
    print("=" * 62)
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
    lock_confirmed = False

    try:
        if getattr(realsense, "use_simulation", False):
            raise BenchFailure("pyrealsense2/T265不可用，拒绝在模拟数据下解锁")

        serial_fc = Lcode.Lprotocol.Serial_fc(port, 460800)
        serial_fc.listen_start(re_fc)
        # 先持续发送零任务指令；T265启动后同一发送线程自动开始喂数。
        serial_fc.send_start(se_fc, realsense, vel_freq=100, cmd_freq=50)
        send_started = True

        read_snapshot = lambda: _read_fc_snapshot(re_fc, serial_fc)
        _wait_for_fc_state(
            read_snapshot,
            unlocked=False,
            label="初始锁桨状态",
            timeout_s=args.lock_timeout_s,
            max_fc_age_s=args.max_fc_age_s,
            max_pwm_age_s=args.max_pwm_age_s,
            confirm_count=args.confirm_count,
        )

        if not realsense.start():
            raise BenchFailure("T265启动失败")
        _wait_for_t265(realsense, args.t265_timeout_s)
        realsense.autoset()
        origin = _position_tuple(realsense)
        print(f"[PASS] T265已启动并且仅校零一次，H点={origin}")

        print("[STEP 1] 发送首次一键起飞指令")
        _set_fc_command(
            se_fc,
            task_sta=1,
            next_task_sign=0,
            height_cm=args.takeoff_height_cm,
        )
        _wait_for_fc_state(
            read_snapshot,
            unlocked=True,
            label="首次解锁",
            timeout_s=args.unlock_timeout_s,
            max_fc_age_s=args.max_fc_age_s,
            max_pwm_age_s=args.max_pwm_age_s,
            confirm_count=args.confirm_count,
        )
        time.sleep(args.motor_observe_s)

        print("[STEP 2] 发送锁桨指令")
        _request_lock_and_confirm(
            se_fc,
            read_snapshot,
            label="第一次锁桨",
            timeout_s=args.lock_timeout_s,
            max_fc_age_s=args.max_fc_age_s,
            max_pwm_age_s=args.max_pwm_age_s,
            confirm_count=args.confirm_count,
        )
        lock_confirmed = True

        print(f"[STEP 3] 锁桨状态保持 {args.locked_hold_s:.1f}s")
        # next_task_sign归零，task_sta仍保持0，只清理飞控任务指令。
        _set_fc_command(se_fc, task_sta=0, next_task_sign=0, height_cm=0)
        hold_deadline = time.monotonic() + args.locked_hold_s
        while time.monotonic() < hold_deadline:
            snapshot = read_snapshot()
            if not _state_matches(
                snapshot,
                unlocked=False,
                max_fc_age_s=args.max_fc_age_s,
                max_pwm_age_s=args.max_pwm_age_s,
            ):
                raise BenchFailure(
                    "5秒停留期间锁桨反馈丢失: "
                    + _format_snapshot(snapshot)
                )
            time.sleep(0.03)

        print("[STEP 4] 产生新的 task_sta 0->1 边沿，请求第二次起飞")
        before_second = _position_tuple(realsense)
        lock_confirmed = False
        _set_fc_command(
            se_fc,
            task_sta=1,
            next_task_sign=0,
            height_cm=args.takeoff_height_cm,
        )
        _wait_for_fc_state(
            read_snapshot,
            unlocked=True,
            label="第二次解锁",
            timeout_s=args.unlock_timeout_s,
            max_fc_age_s=args.max_fc_age_s,
            max_pwm_age_s=args.max_pwm_age_s,
            confirm_count=args.confirm_count,
        )
        after_second = _position_tuple(realsense)
        print(
            "[PASS] T265中途未重启/未再次校零: "
            f"second_before={before_second}, second_after={after_second}"
        )
        time.sleep(args.motor_observe_s)

        print("[STEP 5] 测试结束前再次锁桨")
        _request_lock_and_confirm(
            se_fc,
            read_snapshot,
            label="最终锁桨",
            timeout_s=args.lock_timeout_s,
            max_fc_age_s=args.max_fc_age_s,
            max_pwm_age_s=args.max_pwm_age_s,
            confirm_count=args.confirm_count,
        )
        lock_confirmed = True
        _set_fc_command(se_fc, task_sta=0, next_task_sign=0, height_cm=0)
        print("[RESULT] 通过：首次解锁、锁桨、5秒保持、第二次解锁和最终锁桨均成功")
        return 0

    except KeyboardInterrupt:
        print("\n[ABORT] 用户中断，立即请求锁桨")
        return 130
    except BenchFailure as exc:
        print(f"[FAIL] {exc}")
        return 1
    finally:
        if serial_fc is not None and send_started and not lock_confirmed:
            try:
                print("[SAFETY] 异常收尾：循环发送锁桨指令")
                _request_lock_and_confirm(
                    se_fc,
                    lambda: _read_fc_snapshot(re_fc, serial_fc),
                    label="异常收尾锁桨",
                    timeout_s=args.lock_timeout_s,
                    max_fc_age_s=args.max_fc_age_s,
                    max_pwm_age_s=args.max_pwm_age_s,
                    confirm_count=args.confirm_count,
                )
                lock_confirmed = True
            except Exception as exc:
                print(
                    "[CRITICAL] 无法从遥测确认锁桨，"
                    f"必须现场确认电机已停转: {exc}"
                )
        if serial_fc is not None:
            try:
                _set_fc_command(se_fc, task_sta=0, next_task_sign=0, height_cm=0)
                time.sleep(0.1)
            except Exception:
                pass
            serial_fc.send_end()
            serial_fc.close()
        if realsense.is_running():
            realsense.stop()


if __name__ == "__main__":
    raise SystemExit(main())
