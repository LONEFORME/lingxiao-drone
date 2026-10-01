"""T265 Z轴与飞控下传激光高度的锁桨只读对比工具。

默认只启动T265采集和飞控下行监听，不发送任何数据。显式指定 ``--manual-flight``
后，只向飞控发送现有手动飞行所需的T265速度参考；仍不发送任务指令、解锁或
起飞命令，解锁和飞行完全由遥控器控制。

运行示例：
    python3 t265_laser_height_compare.py --zero-seconds 3

默认日志写入当前目录 ``height_compare_YYYYmmdd_HHMMSS.jsonl``。
"""

import argparse
import json
import math
import os
import time
from dataclasses import dataclass
from pathlib import Path
from statistics import mean

import Lcode.Lprotocol
from Lcode.Logger import logger
from Lcode.global_variable import fc_frame_counter, fc_last_rx_monotonic, lock
from t265 import t265_class


SAMPLE_INTERVAL_S = 0.05
DEFAULT_LASER_MAX_M = 5.0


@dataclass(frozen=True)
class HeightZero:
    laser_m: float
    raw_z_m: float
    filtered_z_m: float


def laser_cm_valid(value, max_m=DEFAULT_LASER_MAX_M):
    """拒绝0、0xFFFFFFFF等无效值，输入为飞控帧中的厘米整数。"""
    try:
        cm = float(value)
    except (TypeError, ValueError):
        return False
    return math.isfinite(cm) and 5.0 < cm <= float(max_m) * 100.0


def compare_height(zero, laser_m, raw_z_m, filtered_z_m, z_sign=1.0):
    """将T265相对位移平移到激光初始绝对高度，返回同步对比量。"""
    raw_delta = float(z_sign) * (float(raw_z_m) - zero.raw_z_m)
    filtered_delta = float(z_sign) * (float(filtered_z_m) - zero.filtered_z_m)
    raw_est = zero.laser_m + raw_delta
    filtered_est = zero.laser_m + filtered_delta
    return {
        "laser_delta_m": float(laser_m) - zero.laser_m,
        "t265_raw_delta_m": raw_delta,
        "t265_filtered_delta_m": filtered_delta,
        "t265_raw_est_height_m": raw_est,
        "t265_filtered_est_height_m": filtered_est,
        "raw_error_m": raw_est - float(laser_m),
        "filtered_error_m": filtered_est - float(laser_m),
    }


def read_fc_snapshot(re_fc):
    with lock:
        laser_cm = re_fc[8] if len(re_fc) > 8 else 0
        frame_id = int(fc_frame_counter.value)
        last_rx = float(fc_last_rx_monotonic.value)
    age_s = time.monotonic() - last_rx if last_rx > 0 else float("inf")
    return laser_cm, frame_id, age_s


def collect_zero(realsense, re_fc, seconds, timeout_s, min_confidence, laser_max_m):
    logger.info("等待稳定零点：激光有效且高度稳定后连续采集%.1f秒", seconds)
    deadline = time.monotonic() + timeout_s
    last_frame_id = -1
    laser_samples = []
    raw_samples = []
    filtered_samples = []
    stable_since = None
    last_status_time = 0.0

    while time.monotonic() < deadline:
        laser_cm, frame_id, frame_age_s = read_fc_snapshot(re_fc)
        confidence = realsense.get_tracking_confidence()
        now = time.monotonic()
        if (
            frame_id != last_frame_id
            and frame_age_s <= 0.5
            and laser_cm_valid(laser_cm, laser_max_m)
            and confidence >= min_confidence
        ):
            last_frame_id = frame_id
            raw_z = float(realsense.get_raw_position()[2])
            filtered_z = float(realsense.get_position()[2])
            if math.isfinite(raw_z) and math.isfinite(filtered_z):
                laser_m = float(laser_cm) / 100.0
                if laser_samples and (
                    abs(laser_m - laser_samples[-1]) > 0.025
                    or abs(filtered_z - filtered_samples[-1]) > 0.025
                ):
                    # 上升/下降期间不取零点；稳定后重新累计完整窗口。
                    laser_samples.clear()
                    raw_samples.clear()
                    filtered_samples.clear()
                    stable_since = None
                if stable_since is None:
                    stable_since = now
                laser_samples.append(laser_m)
                raw_samples.append(raw_z)
                filtered_samples.append(filtered_z)
                if (
                    now - stable_since >= seconds
                    and max(laser_samples) - min(laser_samples) <= 0.04
                    and max(filtered_samples) - min(filtered_samples) <= 0.04
                ):
                    break
        if now - last_status_time >= 0.5:
            stable_elapsed = now - stable_since if stable_since is not None else 0.0
            laser_text = (
                "%.2fm" % (float(laser_cm) / 100.0)
                if laser_cm_valid(laser_cm, laser_max_m)
                else "无效(%s cm)" % laser_cm
            )
            print(
                "\r等待零点：laser=%s conf=%d stable=%.1f/%.1fs"
                % (laser_text, confidence, stable_elapsed, seconds),
                end="",
                flush=True,
            )
            last_status_time = now
        time.sleep(0.01)

    print()
    minimum_samples = max(10, int(seconds * 10))
    if (
        len(laser_samples) < minimum_samples
        or stable_since is None
        or time.monotonic() - stable_since < seconds
    ):
        raise RuntimeError(
            "等待稳定零点超时：有效样本%d；检查激光量程、悬停稳定性和T265置信度"
            % len(laser_samples)
        )
    return HeightZero(mean(laser_samples), mean(raw_samples), mean(filtered_samples))


def parse_args():
    parser = argparse.ArgumentParser(description="锁桨只读比较T265 Z轴和激光高度")
    parser.add_argument("--port", default=os.getenv("DRONE_FC_PORT", "/dev/ttyS6"))
    parser.add_argument("--baud", type=int, default=460800)
    parser.add_argument("--zero-seconds", type=float, default=3.0)
    parser.add_argument(
        "--zero-timeout",
        type=float,
        default=45.0,
        help="等待激光有效且稳定的最长时间；手动实飞可先起飞再自动取零",
    )
    parser.add_argument("--duration", type=float, default=0.0, help="0表示持续到Ctrl+C")
    parser.add_argument("--min-confidence", type=int, default=2, choices=(0, 1, 2, 3))
    parser.add_argument("--laser-max-m", type=float, default=DEFAULT_LASER_MAX_M)
    parser.add_argument("--z-sign", type=int, default=1, choices=(-1, 1))
    parser.add_argument("--allow-simulation", action="store_true")
    parser.add_argument(
        "--manual-flight",
        action="store_true",
        help="仅发送T265速度参考供手动遥控飞行；不会发送任务/解锁/起飞指令",
    )
    parser.add_argument("--output", help="JSONL输出路径；默认按时间生成")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.zero_seconds <= 0:
        raise SystemExit("--zero-seconds必须大于0")

    logger.info("=" * 56)
    if args.manual_flight:
        logger.warning("手动实飞模式：将发送T265速度参考，但不会发送任务/解锁/起飞指令")
    else:
        logger.info("T265/激光高度同步对比：只读、锁桨、不会发送飞控数据")
    logger.info("测试前必须拆桨，确认main.py等程序未占用飞控串口")
    logger.info("=" * 56)

    realsense = t265_class()
    serial_fc = None
    log_file = None
    re_fc = [0] * 14
    samples = []

    try:
        if realsense.use_simulation and not args.allow_simulation:
            raise RuntimeError("未检测到pyrealsense2，拒绝使用模拟T265完成硬件测试")
        if not realsense.start():
            raise RuntimeError("T265启动失败")

        # 等待真实pose流进入并稳定，随后将滤波位置归零；原始位置仍单独记录。
        time.sleep(2.0)
        if realsense.get_tracking_confidence() < args.min_confidence:
            raise RuntimeError("T265置信度不足，重新拔插并等待初始化")
        realsense.autoset()

        serial_fc = Lcode.Lprotocol.Serial_fc(args.port, args.baud)
        serial_fc.listen_start(re_fc)
        if args.manual_flight:
            serial_fc.send_start(t265_obj=realsense, vel_freq=100)
            logger.warning(
                "若地面激光低于量程：现在可手动起飞至约0.5m并稳定悬停，程序将自动取零"
            )
        time.sleep(1.0)

        zero = collect_zero(
            realsense,
            re_fc,
            args.zero_seconds,
            args.zero_timeout,
            args.min_confidence,
            args.laser_max_m,
        )

        output = args.output
        if not output:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            output = str(Path(__file__).resolve().parent / ("height_compare_%s.jsonl" % stamp))
        output_path = Path(output).expanduser().resolve()
        log_file = output_path.open("a", encoding="utf-8")
        log_file.write(json.dumps({
            "event": "height_compare_start",
            "wall_time": time.time(),
            "port": args.port,
            "baud": args.baud,
            "manual_flight": args.manual_flight,
            "z_sign": args.z_sign,
            "zero": {
                "laser_m": zero.laser_m,
                "raw_z_m": zero.raw_z_m,
                "filtered_z_m": zero.filtered_z_m,
            },
        }, ensure_ascii=False) + "\n")
        log_file.flush()

        logger.info(
            "零点完成：laser=%.3fm raw_z=%.3fm filtered_z=%.3fm",
            zero.laser_m,
            zero.raw_z_m,
            zero.filtered_z_m,
        )
        logger.info("日志：%s", output_path)
        logger.info("现在可按0.3/0.6/1.0m逐级抬高；Ctrl+C结束")

        start_mono = time.monotonic()
        last_frame_id = -1
        while args.duration <= 0 or time.monotonic() - start_mono < args.duration:
            laser_cm, frame_id, frame_age_s = read_fc_snapshot(re_fc)
            if frame_id == last_frame_id:
                time.sleep(0.005)
                continue
            last_frame_id = frame_id

            confidence = realsense.get_tracking_confidence()
            raw_z = float(realsense.get_raw_position()[2])
            filtered_z = float(realsense.get_position()[2])
            valid = (
                frame_age_s <= 0.5
                and laser_cm_valid(laser_cm, args.laser_max_m)
                and confidence >= args.min_confidence
                and math.isfinite(raw_z)
                and math.isfinite(filtered_z)
            )
            entry = {
                "t": round(time.time(), 3),
                "elapsed_s": round(time.monotonic() - start_mono, 3),
                "fc_frame": frame_id,
                "fc_age_s": round(frame_age_s, 4) if math.isfinite(frame_age_s) else None,
                "laser_m": round(float(laser_cm) / 100.0, 4) if laser_cm_valid(laser_cm, args.laser_max_m) else None,
                "t265_raw_z_m": round(raw_z, 5),
                "t265_filtered_z_m": round(filtered_z, 5),
                "confidence": confidence,
                "valid": valid,
            }
            if valid:
                comparison = compare_height(
                    zero,
                    float(laser_cm) / 100.0,
                    raw_z,
                    filtered_z,
                    args.z_sign,
                )
                entry.update({key: round(value, 5) for key, value in comparison.items()})
                samples.append(entry)

            log_file.write(json.dumps(entry, ensure_ascii=False) + "\n")
            log_file.flush()

            if valid:
                print(
                    "\rlaser=%+.3fm | T265Δ raw=%+.3fm filt=%+.3fm "
                    "| err raw=%+.3fm filt=%+.3fm | conf=%d"
                    % (
                        entry["laser_m"],
                        entry["t265_raw_delta_m"],
                        entry["t265_filtered_delta_m"],
                        entry["raw_error_m"],
                        entry["filtered_error_m"],
                        confidence,
                    ),
                    end="",
                    flush=True,
                )
            else:
                print(
                    "\r等待有效数据：laser_cm=%s fc_age=%.2fs conf=%d"
                    % (laser_cm, frame_age_s, confidence),
                    end="",
                    flush=True,
                )
            time.sleep(SAMPLE_INTERVAL_S)
    except KeyboardInterrupt:
        print()
        logger.info("用户中断")
    except Exception as exc:
        logger.error("高度对比测试失败：%s", exc)
        raise
    finally:
        if log_file is not None:
            log_file.write(json.dumps({
                "event": "height_compare_end",
                "wall_time": time.time(),
                "valid_samples": len(samples),
            }, ensure_ascii=False) + "\n")
            log_file.close()
        if serial_fc is not None:
            serial_fc.close()
        realsense.stop()
        logger.info("高度对比工具已退出；有效样本=%d", len(samples))


if __name__ == "__main__":
    main()
