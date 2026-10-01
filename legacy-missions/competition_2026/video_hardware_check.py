"""板载摄像头硬件测试 — 不启动飞控，零风险。

测试项：
  1. OpenCV 可用性检查
  2. 摄像头枚举与打开（UVC / 本地相机）
  3. 读取真实帧并验证尺寸
  4. 可选：保存一张临时 JPEG 到指定目录
  5. 帧率统计（读取 N 帧计算平均 FPS）

用法：
    # 自动检测第一个可用摄像头
    python3 video_hardware_test.py

    # 指定摄像头设备和参数
    python3 video_hardware_test.py --source 0 --width 1280 --height 720
    python3 video_hardware_test.py --source /dev/video2

    # 保存一张测试截图
    python3 video_hardware_test.py --snapshot-dir ./camera_test

    # 持续采集并统计 FPS
    python3 video_hardware_test.py --duration 10

失败时退出码非 0，但绝不连接飞控或解锁。
"""
from __future__ import annotations

import argparse
import os
import sys
import time


def _print(name: str, status: str, detail: str) -> None:
    icon = {"PASS": "✓", "FAIL": "✗", "WARN": "⚠", "INFO": "•"}.get(status, "?")
    print(f"  {icon} [{status}] {name}: {detail}")


def check_opencv() -> tuple[bool, object]:
    try:
        import cv2  # type: ignore[import-untyped]
        _print("opencv", "PASS", f"OpenCV {cv2.__version__}")
        return True, cv2
    except ImportError:
        _print("opencv", "FAIL", "OpenCV (cv2) not installed")
        return False, None
    except Exception as exc:
        _print("opencv", "FAIL", f"OpenCV import failed: {exc}")
        return False, None


def enumerate_cameras(cv2: object) -> list[tuple[int, str]]:
    """尝试枚举 /dev/video* 或 0~9 索引的摄像头。"""
    available: list[tuple[int, str]] = []
    # 优先检查 /dev/video*（Linux）
    dev_root = "/dev"
    if os.path.isdir(dev_root):
        for entry in sorted(os.listdir(dev_root)):
            if entry.startswith("video"):
                idx = entry.replace("video", "")
                if idx.isdigit():
                    cap = cv2.VideoCapture(int(idx))
                    if cap.isOpened():
                        name = f"/dev/{entry}"
                        available.append((int(idx), name))
                        cap.release()
    # fallback: 尝试索引 0~4
    seen = {idx for idx, _ in available}
    for idx in range(5):
        if idx not in seen:
            cap = cv2.VideoCapture(idx)
            if cap.isOpened():
                available.append((idx, f"index {idx}"))
                cap.release()
    return available


def test_camera(cv2: object, source: str | int, width: int, height: int, fps_target: int) -> bool:
    """打开指定摄像头，读取一帧并验证基本属性。"""
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        _print("camera_open", "FAIL", f"cannot open source={source}")
        return False

    # 尝试设置参数
    if width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
    if height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
    if fps_target > 0:
        cap.set(cv2.CAP_PROP_FPS, float(fps_target))

    # 等待摄像头稳定
    time.sleep(0.3)

    # 读一帧
    ok, frame = cap.read()
    if not ok or frame is None:
        _print("camera_read", "FAIL", "read() returned false or empty frame")
        cap.release()
        return False

    h, w = frame.shape[:2]
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    _print(
        "camera_read",
        "PASS",
        f"frame {w}x{h} @ ~{actual_fps:.0f} FPS (requested {fps_target})",
    )
    cap.release()
    return True


def test_framerate(cv2: object, source: str | int, duration_s: float) -> bool:
    """持续采集 duration_s 秒，统计平均 FPS。"""
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        _print("framerate", "FAIL", f"cannot open source={source}")
        return False

    count = 0
    t_start = time.monotonic()
    deadline = t_start + duration_s
    while time.monotonic() < deadline:
        ok, _ = cap.read()
        if ok:
            count += 1

    elapsed = time.monotonic() - t_start
    avg_fps = count / elapsed if elapsed > 0 else 0.0
    _print("framerate", "PASS" if avg_fps > 1 else "WARN",
           f"{count} frames in {elapsed:.1f}s = {avg_fps:.1f} FPS")
    cap.release()
    return avg_fps > 1


def test_snapshot(cv2: object, source: str | int, output_dir: str) -> bool:
    """读取一帧并保存为 JPEG，验证写入路径 OK。"""
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        _print("snapshot", "FAIL", f"cannot open source={source}")
        return False

    ok, frame = cap.read()
    cap.release()
    if not ok or frame is None:
        _print("snapshot", "FAIL", "read() failed")
        return False

    os.makedirs(output_dir, exist_ok=True)
    path = os.path.join(output_dir, f"video_test_{time.strftime('%H%M%S')}.jpg")
    temp = path + ".tmp"
    try:
        written = cv2.imwrite(temp, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not written:
            _print("snapshot", "FAIL", "imwrite() returned False")
            return False
        os.replace(temp, path)
        size_kb = os.path.getsize(path) / 1024
        _print("snapshot", "PASS", f"saved {path} ({size_kb:.0f} KB)")
        return True
    except Exception as exc:
        _print("snapshot", "FAIL", str(exc))
        return False
    finally:
        if os.path.exists(temp):
            try:
                os.unlink(temp)
            except OSError:
                pass


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Board-side camera hardware test"
    )
    parser.add_argument("--source", default=None, help="Camera source (index or /dev/videoX)")
    parser.add_argument("--width", type=int, default=640, help="Requested frame width")
    parser.add_argument("--height", type=int, default=480, help="Requested frame height")
    parser.add_argument("--fps", type=int, default=30, help="Requested FPS")
    parser.add_argument(
        "--duration", type=float, default=0,
        help="Collect frames for N seconds and report average FPS (0 = skip)"
    )
    parser.add_argument(
        "--snapshot-dir", default=None,
        help="Save one test snapshot to this directory"
    )
    args = parser.parse_args()

    print("=" * 48)
    print("  板载摄像头测试 (video_hardware_test)")
    print("  " + time.strftime("%Y-%m-%d %H:%M:%S"))
    print("=" * 48)
    print()

    # 1. OpenCV 可用性
    cv_ok, cv2 = check_opencv()
    if not cv_ok:
        print()
        print("  OpenCV 不可用，无法继续。安装: pip install opencv-python")
        return 1

    # 2. 摄像头枚举
    cameras = enumerate_cameras(cv2)
    if not cameras:
        _print("camera_enum", "FAIL", "no available camera found")
        print()
        print("  结果: FAIL (无可用摄像头)")
        return 1
    _print("camera_enum", "PASS", f"{len(cameras)} camera(s) found")
    for idx, name in cameras:
        _print("camera_enum", "INFO", f"  [{idx}] {name}")

    # 3. 选择源并测试
    source: str | int = args.source if args.source is not None else cameras[0][0]
    if isinstance(source, str) and source.isdigit():
        source = int(source)

    _print("camera_select", "INFO", f"testing source={source}")

    checks: list[tuple[str, bool]] = [
        ("单帧读取", test_camera(cv2, source, args.width, args.height, args.fps)),
    ]

    if args.duration > 0:
        checks.append((f"帧率统计({args.duration:.0f}s)", test_framerate(cv2, source, args.duration)))

    if args.snapshot_dir:
        checks.append(("截图保存", test_snapshot(cv2, source, args.snapshot_dir)))

    print()
    print("-" * 48)
    failures = [name for name, ok in checks if not ok]
    if failures:
        print(f"  结果: FAIL ({len(failures)} 项失败)")
        for name in failures:
            print(f"    ✗ {name}")
        print()
        print("  ⚠ 本脚本不启动飞控，失败仅反映摄像头/OpenCV状态。")
        return 1
    else:
        print("  结果: PASS (全部通过)")
        print()
        print("  ✓ 摄像头功能正常。")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
