"""D题地图关键点T265坐标采集工具（不连接飞控、不解锁电机）。"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path

BASIC_DIR = Path(__file__).resolve().parents[1] / "basic"
if str(BASIC_DIR) not in sys.path:
    sys.path.insert(0, str(BASIC_DIR))

from t265 import t265_class  # noqa: E402


@dataclass(frozen=True)
class PoseSample:
    monotonic: float
    x: float
    y: float
    z: float
    raw_x: float
    raw_y: float
    raw_z: float
    yaw_rad: float
    vx: float
    vy: float
    confidence: int
    pose_age_s: float


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = round((len(ordered) - 1) * fraction)
    return ordered[index]


def _circular_mean_deg(values_rad: list[float]) -> float:
    sin_mean = statistics.mean(math.sin(value) for value in values_rad)
    cos_mean = statistics.mean(math.cos(value) for value in values_rad)
    return math.degrees(math.atan2(sin_mean, cos_mean))


def summarize_samples(samples: list[PoseSample], confidence_min: int) -> dict:
    valid = [sample for sample in samples if sample.confidence >= confidence_min]
    if not valid:
        raise RuntimeError("没有达到置信度门限的有效样本")
    xs = [sample.x for sample in valid]
    ys = [sample.y for sample in valid]
    zs = [sample.z for sample in valid]
    speeds = [math.hypot(sample.vx, sample.vy) for sample in valid]
    confidences = [sample.confidence for sample in valid]
    return {
        "sample_count": len(samples),
        "valid_count": len(valid),
        "valid_ratio": len(valid) / len(samples),
        "x_abs_m": statistics.median(xs),
        "y_abs_m": statistics.median(ys),
        "z_abs_m": statistics.median(zs),
        "raw_x_abs_m": statistics.median(sample.raw_x for sample in valid),
        "raw_y_abs_m": statistics.median(sample.raw_y for sample in valid),
        "raw_z_abs_m": statistics.median(sample.raw_z for sample in valid),
        "std_x_cm": statistics.pstdev(xs) * 100.0,
        "std_y_cm": statistics.pstdev(ys) * 100.0,
        "std_z_cm": statistics.pstdev(zs) * 100.0,
        "yaw_deg": _circular_mean_deg([sample.yaw_rad for sample in valid]),
        "speed_median_m_s": statistics.median(speeds),
        "speed_p90_m_s": _percentile(speeds, 0.90),
        "speed_max_m_s": max(speeds),
        "position_span_m": math.hypot(max(xs) - min(xs), max(ys) - min(ys)),
        "confidence_min": min(confidences),
        "confidence_median": statistics.median(confidences),
        "confidence_3_ratio": sum(value == 3 for value in confidences) / len(valid),
        "pose_age_max_s": max(sample.pose_age_s for sample in valid),
    }


class MapPointRecorder:
    def __init__(self, origin_label: str = "H") -> None:
        self.origin_label = origin_label.upper()
        self.origin_xy: tuple[float, float] | None = None
        self.last_abs_xy: tuple[float, float] | None = None
        self.visits: dict[str, int] = {}

    def add_summary(self, label: str, summary: dict, max_step_m: float = 5.0) -> dict:
        label = label.strip().upper()
        if not label:
            raise ValueError("点名不能为空")
        if self.origin_xy is None:
            if label != self.origin_label:
                raise ValueError(f"首个点必须为{self.origin_label}，用于建立零点")
            self.origin_xy = (summary["x_abs_m"], summary["y_abs_m"])
        absolute_xy = (summary["x_abs_m"], summary["y_abs_m"])
        step_m = (
            math.hypot(
                absolute_xy[0] - self.last_abs_xy[0],
                absolute_xy[1] - self.last_abs_xy[1],
            )
            if self.last_abs_xy is not None else 0.0
        )
        if self.last_abs_xy is not None and step_m > max_step_m:
            raise ValueError(
                f"距上一有效点跳变{step_m:.2f}m，超过{max_step_m:.2f}m门限"
            )
        self.last_abs_xy = absolute_xy
        self.visits[label] = self.visits.get(label, 0) + 1
        x_m = summary["x_abs_m"] - self.origin_xy[0]
        y_m = summary["y_abs_m"] - self.origin_xy[1]
        result = {
            "recorded_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "point": label,
            "visit": self.visits[label],
            **summary,
            "x_m": x_m,
            "y_m": y_m,
            "distance_from_origin_m": math.hypot(x_m, y_m),
            "step_from_previous_m": step_m,
            "closure_xy_m": (
                math.hypot(x_m, y_m)
                if label == self.origin_label and self.visits[label] > 1
                else ""
            ),
        }
        return result


def collect_samples(sensor, duration_s: float, sample_hz: float) -> list[PoseSample]:
    samples: list[PoseSample] = []
    deadline = time.monotonic() + duration_s
    interval = 1.0 / sample_hz
    while time.monotonic() < deadline:
        with sensor.lock:
            x, y, z, _roll, _pitch, yaw = sensor.pose_data.tolist()
            raw_x, raw_y, raw_z = sensor.raw_pose_data.tolist()
            vx, vy, _vz = sensor.velocity_data.tolist()
            confidence = int(sensor.last_confidence)
            pose_timestamp = float(sensor.last_pose_monotonic)
        now = time.monotonic()
        pose_age_s = math.inf if pose_timestamp <= 0.0 else now - pose_timestamp
        samples.append(PoseSample(
            now, float(x), float(y), float(z),
            float(raw_x), float(raw_y), float(raw_z), float(yaw),
            float(vx), float(vy), confidence, pose_age_s,
        ))
        time.sleep(interval)
    return samples


def measurement_rejection_reasons(
    summary: dict,
    max_std_cm: float,
    max_speed_m_s: float,
    max_span_m: float,
) -> list[str]:
    reasons = []
    if summary["valid_ratio"] < 0.8:
        reasons.append(f"有效样本比例{summary['valid_ratio']:.0%}<80%")
    xy_std = max(summary["std_x_cm"], summary["std_y_cm"])
    if xy_std > max_std_cm:
        reasons.append(f"XY标准差{xy_std:.1f}cm>{max_std_cm:.1f}cm")
    if summary["speed_p90_m_s"] > max_speed_m_s:
        reasons.append(
            f"静止速度P90={summary['speed_p90_m_s']:.2f}m/s>"
            f"{max_speed_m_s:.2f}m/s"
        )
    if summary["position_span_m"] > max_span_m:
        reasons.append(
            f"采样轨迹跨度{summary['position_span_m']:.2f}m>{max_span_m:.2f}m"
        )
    if summary["pose_age_max_s"] > 0.20:
        reasons.append(f"Pose帧年龄{summary['pose_age_max_s']:.2f}s>0.20s")
    return reasons


def wait_for_confidence(sensor, confidence_min: int, hold_s: float, timeout_s: float) -> None:
    deadline = time.monotonic() + timeout_s
    good_since = None
    while time.monotonic() < deadline:
        confidence = sensor.get_tracking_confidence()
        pose_age_s = sensor.get_pose_age_s()
        if confidence >= confidence_min and pose_age_s <= 0.20:
            if good_since is None:
                good_since = time.monotonic()
            elif time.monotonic() - good_since >= hold_s:
                return
        else:
            good_since = None
        age_text = "NONE" if not math.isfinite(pose_age_s) else f"{pose_age_s:.2f}s"
        print(
            f"\r等待T265：置信度{confidence}/3 Pose年龄={age_text}",
            end="", flush=True,
        )
        time.sleep(0.1)
    print()
    raise RuntimeError("T265置信度等待超时；检查拔插、光照和地面纹理")


def _append_csv(path: Path, record: dict) -> None:
    exists = path.exists()
    with path.open("a", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(record))
        if not exists:
            writer.writeheader()
        writer.writerow(record)


def _append_raw(path: Path, label: str, visit: int, samples: list[PoseSample]) -> None:
    with path.open("a", encoding="utf-8") as handle:
        for sample in samples:
            handle.write(json.dumps({
                "point": label,
                "visit": visit,
                **asdict(sample),
            }, ensure_ascii=False) + "\n")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="交互记录D题地图各点T265坐标")
    parser.add_argument("--sample-seconds", type=float, default=3.0)
    parser.add_argument("--sample-hz", type=float, default=20.0)
    parser.add_argument("--confidence-min", type=int, default=2, choices=(1, 2, 3))
    parser.add_argument("--max-std-cm", type=float, default=2.0)
    parser.add_argument("--max-speed-m-s", type=float, default=0.05)
    parser.add_argument("--max-span-m", type=float, default=0.10)
    parser.add_argument("--max-step-m", type=float, default=5.0)
    parser.add_argument("--output-dir", type=Path, default=Path("map_points"))
    args = parser.parse_args(argv)
    if min(
        args.sample_seconds, args.sample_hz, args.max_std_cm,
        args.max_speed_m_s, args.max_span_m, args.max_step_m,
    ) <= 0:
        parser.error("采样参数和安全门限必须大于0")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = args.output_dir / f"t265_map_points_{stamp}.csv"
    raw_path = args.output_dir / f"t265_map_samples_{stamp}.jsonl"
    sensor = t265_class()
    if not sensor.start():
        raise RuntimeError("T265启动失败")
    recorder = MapPointRecorder("H")
    try:
        wait_for_confidence(sensor, args.confidence_min, 1.0, 12.0)
        print("\nT265已就绪。首个点必须输入H；建议顺序：H A B C_IN C_OUT D H")
        print("每到一点保持无人机水平静止，输入点名并回车；输入q结束。")
        while True:
            label = input("\n点名> ").strip().upper()
            if label in ("Q", "QUIT", "EXIT"):
                break
            if not label:
                continue
            print(f"采集{label}：请保持静止{args.sample_seconds:.1f}秒...")
            samples = collect_samples(sensor, args.sample_seconds, args.sample_hz)
            summary = summarize_samples(samples, args.confidence_min)
            attempt = recorder.visits.get(label, 0) + 1
            _append_raw(raw_path, label, attempt, samples)
            reasons = measurement_rejection_reasons(
                summary, args.max_std_cm, args.max_speed_m_s, args.max_span_m
            )
            if reasons:
                print(f"[拒绝] {label}未写入坐标表：{'；'.join(reasons)}")
                catastrophic = (
                    summary["speed_max_m_s"] > 1.0
                    or summary["position_span_m"] > 0.5
                    or sensor.get_tracking_confidence() == 0
                )
                if catastrophic:
                    print("[致命] T265坐标系疑似失效，本轮自动结束；请返回H并拔插重测")
                    break
                continue
            try:
                record = recorder.add_summary(label, summary, args.max_step_m)
            except ValueError as exc:
                print(f"[拒绝] {exc}")
                continue
            _append_csv(csv_path, record)
            print(
                f"[已记录] {record['point']}#{record['visit']} "
                f"XY=({record['x_m']:+.3f},{record['y_m']:+.3f})m "
                f"std=({record['std_x_cm']:.1f},{record['std_y_cm']:.1f})cm "
                f"conf={record['confidence_median']:.0f}"
            )
            if record["closure_xy_m"] != "":
                print(f"[闭环] 返回H误差={record['closure_xy_m']:.3f}m")
    finally:
        sensor.stop()
    print(f"\n坐标表：{csv_path}\n原始样本：{raw_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
