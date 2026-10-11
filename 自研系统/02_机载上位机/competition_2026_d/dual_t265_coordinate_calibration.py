"""双 T265 场地坐标同步采集工具（不连接飞控、不解锁）。

在无人机 RDK 上运行本模块，同时读取：

* 无人机本机 T265；
* 小车通过 ``CAR_POSITION`` 发送的平台中心场地坐标。

交互命令 ``uav H`` 或 ``car A`` 将对应设备当前静止坐标绑定到题图中的
H/A/B/C/D。采集至少3个不同点后输入 ``fit``，工具分别拟合两端到场地
全局坐标的旋转和平移，并报告尺度、镜像和残差。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import threading
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable

BASIC_DIR = Path(__file__).resolve().parents[1] / "basic"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

from t265 import t265_class  # noqa: E402

from shared.competition_2026_d_protocol import (  # noqa: E402
    Device,
    MessageType,
    PositionFlag,
    seq_is_newer,
    unpack_payload,
)

from .coordinate_alignment import (  # noqa: E402
    OFFICIAL_FIELD_POINTS_M,
    fit_labeled_measurements,
)
from .Lcode.air_ground_link import AirGroundLink, LinkConfig  # noqa: E402


@dataclass(frozen=True)
class CarCoordinateSnapshot:
    session_id: int
    seq: int
    sender_ms: int
    received_monotonic: float
    x_m: float
    y_m: float
    sender_pose_age_s: float
    flags: int

    def total_age_s(self, now: float | None = None) -> float:
        current = time.monotonic() if now is None else float(now)
        return max(0.0, current - self.received_monotonic) + max(
            0.0, self.sender_pose_age_s
        )


class CarCoordinateReceiver:
    """接收校准期间session=0或非0的最新有效CAR_POSITION。"""

    def __init__(self, link: AirGroundLink) -> None:
        self._lock = threading.Lock()
        self._latest: CarCoordinateSnapshot | None = None
        self._last_session: int | None = None
        self._last_seq: int | None = None
        self.accepted_frames = 0
        self.rejected_frames = 0
        link.add_callback(self._on_frame)

    def _on_frame(self, frame) -> None:
        if (
            int(frame.message_type) != int(MessageType.CAR_POSITION)
            or int(frame.source) != int(Device.CAR)
            or int(frame.dest)
            not in (int(Device.UAV), int(Device.BROADCAST))
        ):
            return
        try:
            x_mm, y_mm, pose_age_ms, flags = unpack_payload(
                MessageType.CAR_POSITION,
                frame.payload,
            )
        except ValueError:
            self.rejected_frames += 1
            return
        if int(flags) & ~0x000F:
            self.rejected_frames += 1
            return
        required = int(PositionFlag.CAR_POSE_VALID)
        if int(flags) & required != required:
            self.rejected_frames += 1
            return
        session_valid = bool(int(flags) & int(PositionFlag.SESSION_VALID))
        if session_valid != (int(frame.session_id) != 0):
            self.rejected_frames += 1
            return
        with self._lock:
            session = int(frame.session_id)
            if self._last_session == session and self._last_seq is not None:
                if not seq_is_newer(int(frame.seq), self._last_seq):
                    self.rejected_frames += 1
                    return
            snapshot = CarCoordinateSnapshot(
                session_id=session,
                seq=int(frame.seq),
                sender_ms=int(frame.sender_ms),
                received_monotonic=time.monotonic(),
                x_m=float(x_mm) / 1000.0,
                y_m=float(y_mm) / 1000.0,
                sender_pose_age_s=float(pose_age_ms) / 1000.0,
                flags=int(flags),
            )
            self._latest = snapshot
            self._last_session = session
            self._last_seq = int(frame.seq)
            self.accepted_frames += 1

    def latest(self) -> CarCoordinateSnapshot | None:
        with self._lock:
            return self._latest


@dataclass(frozen=True)
class StaticSummary:
    count: int
    x_m: float
    y_m: float
    std_x_cm: float
    std_y_cm: float
    span_cm: float
    max_age_s: float
    min_confidence: int | None


def summarize_xy_samples(
    samples: list[tuple[float, float, float, int | None]],
) -> StaticSummary:
    if len(samples) < 2:
        raise RuntimeError("有效样本不足")
    xs = [sample[0] for sample in samples]
    ys = [sample[1] for sample in samples]
    return StaticSummary(
        count=len(samples),
        x_m=statistics.median(xs),
        y_m=statistics.median(ys),
        std_x_cm=statistics.pstdev(xs) * 100.0,
        std_y_cm=statistics.pstdev(ys) * 100.0,
        span_cm=math.hypot(max(xs) - min(xs), max(ys) - min(ys)) * 100.0,
        max_age_s=max(sample[2] for sample in samples),
        min_confidence=(
            min(
                int(sample[3])
                for sample in samples
                if sample[3] is not None
            )
            if any(sample[3] is not None for sample in samples)
            else None
        ),
    )


def collect_uav_samples(
    sensor,
    duration_s: float,
    sample_hz: float,
    confidence_min: int,
) -> list[tuple[float, float, float, int | None]]:
    samples = []
    interval = 1.0 / sample_hz
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        now = time.monotonic()
        confidence = sensor.get_tracking_confidence()
        age_s = sensor.get_pose_age_s(now)
        if confidence >= confidence_min and age_s <= 0.20:
            position = sensor.get_position()
            samples.append(
                (
                    float(position[0]),
                    float(position[1]),
                    float(age_s),
                    int(confidence),
                )
            )
        time.sleep(interval)
    return samples


def collect_car_samples(
    receiver: CarCoordinateReceiver,
    duration_s: float,
    max_age_s: float,
) -> list[tuple[float, float, float, int | None]]:
    samples = []
    deadline = time.monotonic() + duration_s
    last_key: tuple[int, int] | None = None
    while time.monotonic() < deadline:
        snapshot = receiver.latest()
        if snapshot is not None:
            key = (snapshot.session_id, snapshot.seq)
            age_s = snapshot.total_age_s()
            fresh_flag = bool(
                snapshot.flags & int(PositionFlag.CAR_POSE_FRESH)
            )
            if key != last_key and fresh_flag and age_s <= max_age_s:
                samples.append(
                    (snapshot.x_m, snapshot.y_m, age_s, None)
                )
                last_key = key
        time.sleep(0.01)
    return samples


def _wait_uav_ready(
    sensor,
    confidence_min: int,
    timeout_s: float,
) -> None:
    deadline = time.monotonic() + timeout_s
    stable_since = None
    while time.monotonic() < deadline:
        confidence = sensor.get_tracking_confidence()
        age_s = sensor.get_pose_age_s()
        if confidence >= confidence_min and age_s <= 0.20:
            stable_since = stable_since or time.monotonic()
            if time.monotonic() - stable_since >= 1.0:
                return
        else:
            stable_since = None
        age_text = "NONE" if not math.isfinite(age_s) else f"{age_s:.2f}s"
        print(
            f"\r等待无人机T265：confidence={confidence}/3 age={age_text}",
            end="",
            flush=True,
        )
        time.sleep(0.10)
    print()
    raise RuntimeError("无人机T265就绪超时")


def _append_csv(path: Path, record: dict) -> None:
    exists = path.exists()
    with path.open("a", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(record))
        if not exists:
            writer.writeheader()
        writer.writerow(record)


def _load_field_points(path: Path | None) -> dict[str, tuple[float, float]]:
    if path is None:
        return dict(OFFICIAL_FIELD_POINTS_M)
    data = json.loads(path.read_text(encoding="utf-8"))
    result = {}
    for label, value in data.items():
        if (
            not isinstance(value, list)
            or len(value) != 2
            or not all(math.isfinite(float(item)) for item in value)
        ):
            raise ValueError(f"场地点{label}必须为两个有限数值")
        result[str(label).upper()] = (float(value[0]), float(value[1]))
    return result


def _aggregate_measurements(
    records: list[dict],
    source: str,
) -> dict[str, tuple[float, float]]:
    grouped: dict[str, list[tuple[float, float]]] = {}
    for record in records:
        if record["source"] != source:
            continue
        grouped.setdefault(record["point"], []).append(
            (record["x_m"], record["y_m"])
        )
    return {
        label: (
            statistics.median(point[0] for point in points),
            statistics.median(point[1] for point in points),
        )
        for label, points in grouped.items()
    }


def _print_fit(
    source: str,
    measurements: dict[str, tuple[float, float]],
    field_points: dict[str, tuple[float, float]],
) -> tuple[dict, dict]:
    transform, residuals = fit_labeled_measurements(
        measurements,
        field_points,
    )
    print(
        f"\n[{source.upper()}拟合] "
        f"旋转={transform.rotation_deg:+.2f}deg "
        f"平移=({transform.translation_x_m:+.3f},"
        f"{transform.translation_y_m:+.3f})m "
        f"尺度诊断={transform.scale_diagnostic:.4f} "
        f"RMS={transform.rms_error_m*100:.1f}cm "
        f"MAX={transform.max_error_m*100:.1f}cm"
    )
    if transform.reflection_suspected:
        print(
            "[警告] 镜像拟合显著更好，疑似X/Y方向、左右定义或符号错误"
        )
    if abs(transform.scale_diagnostic - 1.0) > 0.02:
        print("[警告] 尺度偏离1超过2%，检查单位、滤波和场地图尺寸")
    for label, residual in residuals.items():
        print(
            f"  {label}: 预测=({residual['predicted_x_m']:+.3f},"
            f"{residual['predicted_y_m']:+.3f})m "
            f"期望=({residual['expected_x_m']:+.3f},"
            f"{residual['expected_y_m']:+.3f})m "
            f"误差={residual['error_m']*100:.1f}cm"
        )
    return transform.to_dict(), residuals


def _record_source(
    *,
    source: str,
    point: str,
    collector: Callable[[], list[tuple[float, float, float, int | None]]],
    records: list[dict],
    field_points: dict[str, tuple[float, float]],
    csv_path: Path,
    max_std_cm: float,
    max_span_cm: float,
) -> None:
    samples = collector()
    summary = summarize_xy_samples(samples)
    if (
        max(summary.std_x_cm, summary.std_y_cm) > max_std_cm
        or summary.span_cm > max_span_cm
    ):
        print(
            f"[拒绝] {source.upper()} {point}静止性不足："
            f"std=({summary.std_x_cm:.1f},{summary.std_y_cm:.1f})cm "
            f"span={summary.span_cm:.1f}cm"
        )
        return
    visit = 1 + sum(
        record["source"] == source and record["point"] == point
        for record in records
    )
    expected = field_points[point]
    record = {
        "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "source": source,
        "point": point,
        "visit": visit,
        **asdict(summary),
        "expected_x_m": expected[0],
        "expected_y_m": expected[1],
    }
    records.append(record)
    _append_csv(csv_path, record)
    print(
        f"[已记录] {source.upper()} {point}#{visit} "
        f"XY=({summary.x_m:+.3f},{summary.y_m:+.3f})m "
        f"std=({summary.std_x_cm:.1f},{summary.std_y_cm:.1f})cm "
        f"样本={summary.count}"
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="双T265场地坐标同步采集与刚体对齐（不连接飞控）"
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).with_name("config.json"),
    )
    parser.add_argument("--field-points-json", type=Path)
    parser.add_argument("--sample-seconds", type=float, default=3.0)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--confidence-min", type=int, default=2)
    parser.add_argument("--car-max-age-s", type=float, default=0.30)
    parser.add_argument("--max-std-cm", type=float, default=2.0)
    parser.add_argument("--max-span-cm", type=float, default=8.0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("map_points"),
    )
    args = parser.parse_args(argv)
    if (
        args.sample_seconds <= 0.0
        or args.sample_hz <= 0.0
        or args.car_max_age_s <= 0.0
        or args.max_std_cm <= 0.0
        or args.max_span_cm <= 0.0
    ):
        parser.error("采样时间、频率和门限必须大于0")
    if args.confidence_min not in (1, 2, 3):
        parser.error("--confidence-min必须为1、2或3")

    config = json.loads(args.config.read_text(encoding="utf-8"))
    bluetooth = config["bluetooth"]
    field_points = _load_field_points(args.field_points_json)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = args.output_dir / f"dual_t265_points_{stamp}.csv"
    result_path = args.output_dir / f"dual_t265_alignment_{stamp}.json"

    link = AirGroundLink(
        LinkConfig(
            port=str(bluetooth["port"]),
            baudrate=int(bluetooth["baudrate"]),
            write_timeout_s=float(bluetooth.get("write_timeout_s", 0.20)),
            ack_timeout_s=float(bluetooth["ack_timeout_s"]),
            max_retries=int(bluetooth["max_retries"]),
            max_consecutive_tx_errors=int(
                bluetooth.get("max_consecutive_tx_errors", 3)
            ),
        )
    )
    if not link.start():
        raise RuntimeError("蓝牙链路启动失败")
    receiver = CarCoordinateReceiver(link)
    sensor = t265_class()
    records: list[dict] = []
    try:
        if not sensor.start():
            raise RuntimeError("无人机T265启动失败")
        _wait_uav_ready(sensor, args.confidence_min, 12.0)
        sensor.autoset()
        print("\n双T265校准工具已就绪；不会连接飞控，也不会解锁。")
        print("题图场地点（全局坐标，m）：")
        for label, xy_m in field_points.items():
            print(f"  {label}=({xy_m[0]:.3f},{xy_m[1]:.3f})")
        print(
            "\n命令：uav H / uav A / car A / car B / ...；"
            "fit拟合；status看链路；q退出。"
        )
        print(
            "建议先保持无人机T265不中断，依次记录UAV H/A/B/C/D/H；"
            "再保持小车T265不中断，记录CAR A/B/C/D/A。"
        )
        while True:
            command = input("\n校准> ").strip()
            if not command:
                continue
            lowered = command.lower()
            if lowered in ("q", "quit", "exit"):
                break
            if lowered == "status":
                snapshot = receiver.latest()
                if snapshot is None:
                    print(
                        "CAR_POSITION尚未收到；"
                        f"accepted={receiver.accepted_frames} "
                        f"rejected={receiver.rejected_frames}"
                    )
                else:
                    print(
                        f"CAR_POSITION session={snapshot.session_id} "
                        f"seq={snapshot.seq} "
                        f"XY=({snapshot.x_m:+.3f},{snapshot.y_m:+.3f})m "
                        f"age={snapshot.total_age_s():.3f}s "
                        f"accepted={receiver.accepted_frames} "
                        f"rejected={receiver.rejected_frames}"
                    )
                print(
                    f"UAV T265 confidence={sensor.get_tracking_confidence()}/3 "
                    f"age={sensor.get_pose_age_s():.3f}s"
                )
                continue
            if lowered == "fit":
                result = {
                    "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "field_points_m": field_points,
                    "sources": {},
                }
                for source in ("uav", "car"):
                    measurements = _aggregate_measurements(records, source)
                    try:
                        transform, residuals = _print_fit(
                            source,
                            measurements,
                            field_points,
                        )
                    except ValueError as exc:
                        print(f"[{source.upper()}未拟合] {exc}")
                        continue
                    result["sources"][source] = {
                        "measurements_m": measurements,
                        "transform": transform,
                        "residuals": residuals,
                    }
                result_path.write_text(
                    json.dumps(result, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )
                print(f"\n拟合结果已写入：{result_path}")
                continue

            parts = command.split()
            if len(parts) != 2 or parts[0].lower() not in ("uav", "car"):
                print("命令格式错误，例如：uav H、car A、fit、status、q")
                continue
            source = parts[0].lower()
            point = parts[1].upper()
            if point not in field_points:
                print(f"未知场地点{point}；可用：{', '.join(field_points)}")
                continue
            print(
                f"采集{source.upper()} {point}："
                f"请保持对应设备静止{args.sample_seconds:.1f}秒..."
            )
            if source == "uav":
                collector = lambda: collect_uav_samples(
                    sensor,
                    args.sample_seconds,
                    args.sample_hz,
                    args.confidence_min,
                )
            else:
                collector = lambda: collect_car_samples(
                    receiver,
                    args.sample_seconds,
                    args.car_max_age_s,
                )
            try:
                _record_source(
                    source=source,
                    point=point,
                    collector=collector,
                    records=records,
                    field_points=field_points,
                    csv_path=csv_path,
                    max_std_cm=args.max_std_cm,
                    max_span_cm=args.max_span_cm,
                )
            except RuntimeError as exc:
                print(f"[拒绝] {exc}")
    finally:
        sensor.stop()
        link.close()
    print(f"\n采样表：{csv_path}")
    if result_path.exists():
        print(f"拟合结果：{result_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
