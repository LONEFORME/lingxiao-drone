"""通信联调诊断脚本。

独立运行，不依赖飞控/T265/视觉。只连接蓝牙链路，监听并解析所有帧，
实时显示帧内容和统计。用于验证小车端是否正确发送 CAR_STATE 扩展字段。

用法（在板子上）：
    python -m drone_control.competition_2026_d.comm_diagnostic
    python comm_diagnostic.py   # 直接运行

Ctrl+C 退出时打印总结。
"""

from __future__ import annotations

import argparse
import json
import signal
import sys
import time
from collections import defaultdict
from pathlib import Path

BASIC_DIR = Path(__file__).resolve().parents[1] / "basic"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

from shared.competition_2026_d_protocol import (  # noqa: E402
    CarSegment,
    CarStateFlag,
    CarStatePayload,
    Device,
    Flag,
    Frame,
    MessageType,
    PositionFlag,
    UavPhase,
    pack_payload,
    unpack_car_state,
    unpack_payload,
)

from .Lcode.air_ground_link import AirGroundLink, LinkConfig  # noqa: E402


_TYPE_NAMES = {int(k): k.name for k in MessageType}
_SEGMENT_NAMES = {int(k): k.name for k in CarSegment}


class FrameStats:
    def __init__(self) -> None:
        self.counts: dict[int, int] = defaultdict(int)
        self.first_ts: dict[int, float] = {}
        self.last_ts: dict[int, float] = {}
        self.car_state_basic = 0
        self.car_state_extended = 0
        self.car_state_vx_sum = 0.0
        self.car_state_vy_sum = 0.0
        self.errors = 0

    def record(self, msg_type: int, now: float) -> None:
        self.counts[msg_type] += 1
        if msg_type not in self.first_ts:
            self.first_ts[msg_type] = now
        self.last_ts[msg_type] = now

    def summary(self) -> str:
        lines = ["==== 通信诊断总结 ===="]
        if not self.counts:
            lines.append("  未收到任何帧")
            return "\n".join(lines)
        for msg_type, count in sorted(self.counts.items()):
            name = _TYPE_NAMES.get(msg_type, f"0x{msg_type:02X}")
            first = self.first_ts[msg_type]
            last = self.last_ts[msg_type]
            span = max(last - first, 1e-6)
            hz = count / span
            lines.append(
                f"  {name:20s}  count={count:5d}  span={span:6.1f}s  "
                f"avg={hz:5.1f}Hz"
            )
        if self.car_state_basic + self.car_state_extended > 0:
            total = self.car_state_basic + self.car_state_extended
            lines.append("")
            lines.append(f"  CAR_STATE 基础格式(9B):   {self.car_state_basic}")
            lines.append(f"  CAR_STATE 扩展格式(13B):  {self.car_state_extended}")
            if self.car_state_extended > 0:
                avg_vx = self.car_state_vx_sum / self.car_state_extended
                avg_vy = self.car_state_vy_sum / self.car_state_extended
                lines.append(
                    f"  扩展字段均值 vx={avg_vx:+.1f}mm/s  "
                    f"vy={avg_vy:+.1f}mm/s"
                )
            verdict = "OK" if self.car_state_extended > 0 else "MISSING"
            lines.append(f"  任务二速度前馈: {verdict}")
        if self.errors:
            lines.append(f"\n  解析错误: {self.errors}")
        return "\n".join(lines)


def _format_flags(flags: int, flag_enum) -> str:
    if flags == 0:
        return "none"
    names = []
    for f in flag_enum:
        if flags & int(f):
            names.append(f.name)
    return "|".join(names) if names else f"0x{flags:X}"


def _format_car_state(payload: bytes, stats: FrameStats) -> str:
    try:
        state = unpack_car_state(payload)
    except ValueError as exc:
        stats.errors += 1
        hex_str = payload.hex(" ")
        return (
            f"  CAR_STATE 解析失败: {exc} (len={len(payload)})\n"
            f"  payload_hex: {hex_str}"
        )

    if state.has_world_velocity:
        stats.car_state_extended += 1
        stats.car_state_vx_sum += state.vx_mm_s
        stats.car_state_vy_sum += state.vy_mm_s
        fmt = "扩展(13B)"
        vel = f"  vx={state.vx_mm_s:+5d}mm/s  vy={state.vy_mm_s:+5d}mm/s"
    else:
        stats.car_state_basic += 1
        fmt = "基础(9B)"
        vel = "  vx/vy=MISSING"

    seg = _SEGMENT_NAMES.get(state.segment, f"?{state.segment}")
    flags = _format_flags(state.state_flags, CarStateFlag)
    return (
        f"  CAR_STATE [{fmt}]  seg={seg}  "
        f"speed={state.speed_mm_s:4d}mm/s  "
        f"hdg={state.heading_cdeg:5d}(0.01deg)  "
        f"track={state.track_s_mm:6d}mm  "
        f"flags={flags}{vel}"
    )


def _format_car_position(payload: bytes) -> str:
    try:
        x_mm, y_mm, pose_age_ms, flags = unpack_payload(
            MessageType.CAR_POSITION, payload
        )
    except ValueError as exc:
        return f"  CAR_POSITION 解析失败: {exc}"
    flag_str = _format_flags(flags, PositionFlag)
    return (
        f"  CAR_POSITION  x={x_mm/1000:+.3f}m  y={y_mm/1000:+.3f}m  "
        f"pose_age={pose_age_ms}ms  flags={flag_str}"
    )


def _format_car_start(frame: Frame) -> str:
    try:
        task_mode, config_hash = unpack_payload(
            MessageType.CAR_START, frame.payload
        )
    except ValueError as exc:
        return f"  CAR_START 解析失败: {exc}"
    return (
        f"  CAR_START  task_mode={task_mode}  "
        f"config_hash=0x{config_hash:08X}  session={frame.session_id}"
    )


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="通信联调诊断")
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("config.json"),
    )
    parser.add_argument(
        "--no-ready-broadcast",
        action="store_true",
        help="不周期广播UAV_READY（默认每秒广播）",
    )
    parser.add_argument(
        "--no-uav-state",
        action="store_true",
        help="不模拟发送UAV_STATE/UAV_EVENT（默认收到CAR_START后模拟）",
    )
    args = parser.parse_args(argv)

    data = json.loads(args.config.read_bytes())
    bt = data["bluetooth"]
    link = AirGroundLink(
        LinkConfig(
            port=str(bt["port"]),
            baudrate=int(bt["baudrate"]),
            write_timeout_s=float(bt.get("write_timeout_s", 0.20)),
            ack_timeout_s=float(bt["ack_timeout_s"]),
            max_retries=int(bt["max_retries"]),
            max_consecutive_tx_errors=int(
                bt.get("max_consecutive_tx_errors", 3)
            ),
        )
    )
    stats = FrameStats()
    sim_state = {
        "enabled": not args.no_uav_state,
        "session": 0,
        "started_at": 0.0,
        "last_state_at": 0.0,
        "last_phase": -1,
    }
    sim_phases = [
        (int(UavPhase.TAKEOFF), 3.0, (0.0, 0.0), 1.5, "TAKEOFF"),
        (int(UavPhase.HOVER), 3.0, (0.0, 0.0), 1.5, "HOVER"),
        (int(UavPhase.INTERCEPT), 5.0, (0.375, 1.75), 1.0, "INTERCEPT"),
        (int(UavPhase.FORMATION_FOLLOW), 12.0, (1.0, 2.0), 1.0, "FOLLOW"),
        (int(UavPhase.DROP), 4.0, (1.875, 2.25), 0.4, "DROP"),
        (int(UavPhase.RETURN_H), 8.0, (0.0, 0.0), 1.5, "RETURN"),
        (int(UavPhase.LAND_H), 3.0, (0.0, 0.0), 0.15, "LAND"),
        (int(UavPhase.COMPLETE), 999.0, (0.0, 0.0), 0.15, "COMPLETE"),
    ]
    stop = {"flag": False}

    def on_frame(frame: Frame) -> None:
        now = time.monotonic()
        stats.record(frame.message_type, now)
        name = _TYPE_NAMES.get(frame.message_type, f"0x{frame.message_type:02X}")
        ts = time.strftime("%H:%M:%S")
        print(f"[{ts}] {name}  src={frame.source} dst={frame.dest} "
              f"seq={frame.seq} session={frame.session_id} "
              f"flags=0x{int(frame.flags):02X} len={len(frame.payload)}")
        if frame.message_type == int(MessageType.CAR_START):
            print(_format_car_start(frame))
        elif frame.message_type == int(MessageType.CAR_STATE):
            print(_format_car_state(frame.payload, stats))
        elif frame.message_type == int(MessageType.CAR_POSITION):
            print(_format_car_position(frame.payload))
        elif frame.message_type == int(MessageType.ACK):
            print("  ACK")
        elif frame.message_type == int(MessageType.HEARTBEAT):
            print("  HEARTBEAT")
        else:
            print(f"  payload_hex={frame.payload.hex()}")

        # 自动回 ACK
        if int(frame.flags) & int(Flag.ACK_REQUIRED):
            link.acknowledge(frame, result=0)
            if frame.message_type == int(MessageType.CAR_START):
                print(f"  -> 已回复 ACK (result=0)")

        # 收到 CAR_START 启动模拟
        if (
            frame.message_type == int(MessageType.CAR_START)
            and sim_state["enabled"]
            and sim_state["started_at"] == 0.0
        ):
            sim_state["session"] = int(frame.session_id)
            sim_state["started_at"] = now
            sim_state["last_phase"] = -1
            sim_state["last_state_at"] = 0.0
            print(f"  [SIM] 开始模拟 UAV_STATE/UAV_EVENT "
                  f"(session={frame.session_id})")

    link.add_callback(on_frame)

    if not link.start():
        print("蓝牙链路启动失败", file=sys.stderr)
        sys.exit(1)

    print(f"通信诊断已启动，监听 {bt['port']} @ {bt['baudrate']} baud")
    print("等待小车发送帧... (Ctrl+C 退出)\n")

    ready_mask = 0xFF
    ready_bits = 0x0F
    config_hash = 0
    next_ready = 0.0

    def _signal_handler(sig, frame):
        stop["flag"] = True
        print("\n正在退出...")
    signal.signal(signal.SIGINT, _signal_handler)

    try:
        while not stop["flag"]:
            now = time.monotonic()
            if not args.no_ready_broadcast and now >= next_ready:
                link.publish(
                    MessageType.UAV_READY,
                    pack_payload(
                        MessageType.UAV_READY,
                        (ready_mask, ready_bits, config_hash),
                    ),
                    session_id=0,
                    dest=Device.CAR,
                )
                next_ready = now + 1.0

            # 模拟 UAV_STATE/UAV_EVENT
            if sim_state["enabled"] and sim_state["started_at"] > 0.0:
                elapsed = now - sim_state["started_at"]
                cumulative = 0.0
                phase_idx = 0
                for i, (_, dur, _, _, _) in enumerate(sim_phases):
                    if elapsed < cumulative + dur or i == len(sim_phases) - 1:
                        phase_idx = i
                        break
                    cumulative += dur
                phase_enum, _, hover_xy, height, label = sim_phases[phase_idx]
                if phase_enum != sim_state["last_phase"]:
                    sim_state["last_phase"] = phase_enum
                    link.publish(
                        MessageType.UAV_EVENT,
                        pack_payload(
                            MessageType.UAV_EVENT,
                            (phase_enum, int(elapsed * 1000)),
                        ),
                        session_id=sim_state["session"],
                        dest=Device.CAR,
                        flags=Flag.ACK_REQUIRED | Flag.EVENT,
                    )
                    print(f"  [SIM] -> UAV_EVENT phase={label}")
                if now - sim_state["last_state_at"] >= 0.10:
                    sim_state["last_state_at"] = now
                    link.publish(
                        MessageType.UAV_STATE,
                        pack_payload(
                            MessageType.UAV_STATE,
                            (int(hover_xy[0] * 1000),
                             int(hover_xy[1] * 1000),
                             int(height * 1000)),
                        ),
                        session_id=sim_state["session"],
                        dest=Device.CAR,
                    )
            time.sleep(0.03)
    finally:
        link.close()
        print()
        print(stats.summary())


if __name__ == "__main__":
    main()
