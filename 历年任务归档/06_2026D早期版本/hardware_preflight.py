"""板载只读硬件预检 — 不发送任何解锁/起飞指令，零风险，可直接跑。

检查项：
  1. Python 版本
  2. GPIO 模块可导入（wiringpi / RPi.GPIO / lgpio）
  3. T265 是否可通过 pyrealsense2 枚举
  4. 飞控串口可打开并短时间收到有效帧
  5. 磁盘剩余空间
  6. 可执行文件（pytest, git, python3）可用性

用法：
    python3 hardware_preflight.py
    python3 hardware_preflight.py --fc-port /dev/ttyS6
    python3 hardware_preflight.py --duration 10

失败时退出码非 0，但绝不尝试解锁或起飞。
"""
from __future__ import annotations

import argparse
import os
import shutil
import sys
import time


def _print(name: str, status: str, detail: str) -> None:
    icon = {"PASS": "✓", "FAIL": "✗", "WARN": "⚠", "INFO": "•"}.get(status, "?")
    print(f"  {icon} [{status}] {name}: {detail}")


def check_python_version() -> bool:
    ok = sys.version_info >= (3, 10)
    _print(
        "python_version",
        "PASS" if ok else "FAIL",
        f"{sys.version} (need >=3.10)",
    )
    return ok


def check_gpio_module() -> bool:
    for name, import_path in [
        ("RPi.GPIO", "RPi.GPIO"),
        ("wiringpi", "wiringpi"),
        ("lgpio", "lgpio"),
    ]:
        try:
            __import__(import_path)
            _print("gpio_module", "PASS", f"{name} available")
            return True
        except ImportError:
            continue
        except Exception as exc:
            _print("gpio_module", "WARN", f"{name} import failed: {exc}")
            continue
    _print("gpio_module", "WARN", "no GPIO module found (expected on dev PC)")
    return False


def check_t265() -> bool:
    try:
        import pyrealsense2 as rs  # type: ignore[import-untyped]
    except ImportError:
        _print("t265", "WARN", "pyrealsense2 not installed")
        return False

    ctx = rs.context()
    devices = list(ctx.devices)
    if not devices:
        _print("t265", "FAIL", "no RealSense device enumerated")
        return False
    for dev in devices:
        _print(
            "t265",
            "INFO",
            f"  {dev.get_info(rs.camera_info.name)} "
            f"(serial: {dev.get_info(rs.camera_info.serial_number)})",
        )
    _print("t265", "PASS", f"{len(devices)} device(s) enumerated")
    return True


def check_fc_serial(port: str, duration_s: float = 5.0) -> bool:
    try:
        sys.path.insert(0, os.path.dirname(__file__))
        from Lcode.Lprotocol import Serial_fc
    except ImportError as exc:
        _print("fc_serial", "FAIL", f"cannot import Lprotocol: {exc}")
        return False

    serial_fc = None
    try:
        serial_fc = Serial_fc(port, 460800)
        re_fc = [0] * 14
        serial_fc.listen_start(re_fc)
        _print("fc_serial", "INFO", f"listening on {port} for {duration_s:.0f}s...")

        t_start = time.time()
        data_seen = False
        samples = 0
        while time.time() - t_start < duration_s:
            # 检查 re_fc 是否已被飞控数据填充（任意非零字段都算有效收帧）
            if len(re_fc) > 5 and any(v != 0 for v in re_fc[1:6]):
                data_seen = True
                samples += 1
            time.sleep(0.05)

        if not data_seen:
            _print("fc_serial", "FAIL", f"no valid frame received on {port} in {duration_s:.0f}s")
            return False
        _print("fc_serial", "PASS", f"{samples} samples received on {port}")
        return True
    except Exception as exc:
        _print("fc_serial", "FAIL", f"serial error on {port}: {exc}")
        return False
    finally:
        if serial_fc is not None:
            try:
                serial_fc.listen_end()
            except Exception:
                pass


def check_disk_space(min_free_mb: int = 256) -> bool:
    try:
        usage = shutil.disk_usage(os.path.dirname(__file__) or ".")
        free_mb = usage.free // (1024 * 1024)
        ok = free_mb >= min_free_mb
        _print(
            "disk_space",
            "PASS" if ok else "FAIL",
            f"{free_mb} MiB free (require {min_free_mb} MiB)",
        )
        return ok
    except OSError as exc:
        _print("disk_space", "FAIL", str(exc))
        return False


def check_executables() -> bool:
    """检查必需的可执行文件。pytest 缺失只警告不阻断。"""
    required = ("git", "python3")
    optional = ("pytest",)
    all_ok = True
    for exe in required:
        path = shutil.which(exe)
        if path:
            _print(f"executable:{exe}", "PASS", path)
        else:
            _print(f"executable:{exe}", "FAIL", f"{exe} not found in PATH")
            all_ok = False
    for exe in optional:
        path = shutil.which(exe)
        if path:
            _print(f"executable:{exe}", "PASS", path)
        else:
            _print(f"executable:{exe}", "WARN", f"{exe} not found in PATH (optional)")
    return all_ok


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Board-side read-only hardware preflight check"
    )
    parser.add_argument(
        "--fc-port",
        default=os.getenv("DRONE_FC_PORT", "/dev/ttyS6"),
        help="Flight controller serial port",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=5.0,
        help="Seconds to listen for FC frames",
    )
    parser.add_argument(
        "--min-free-mb",
        type=int,
        default=256,
        help="Minimum free disk space in MiB",
    )
    parser.add_argument(
        "--skip-t265",
        action="store_true",
        help="Skip T265 enumeration check",
    )
    parser.add_argument(
        "--skip-fc",
        action="store_true",
        help="Skip flight controller serial check",
    )
    args = parser.parse_args()

    print("=" * 48)
    print("  板载只读硬件预检 (hardware_preflight)")
    print("  " + time.strftime("%Y-%m-%d %H:%M:%S"))
    print("=" * 48)
    print()

    checks = [
        ("Python版本", check_python_version()),
        ("GPIO模块", check_gpio_module()),
        ("可执行文件", check_executables()),
        ("磁盘空间", check_disk_space(args.min_free_mb)),
    ]

    if not args.skip_t265:
        checks.append(("T265枚举", check_t265()))
    if not args.skip_fc:
        checks.append((f"飞控串口({args.fc_port})", check_fc_serial(args.fc_port, args.duration)))

    print()
    print("-" * 48)
    failures = [name for name, ok in checks if not ok]
    if failures:
        print(f"  结果: FAIL ({len(failures)} 项失败)")
        for name in failures:
            print(f"    ✗ {name}")
        print()
        print("  ⚠ 本脚本不发送任何解锁/起飞指令，失败仅反映硬件/环境状态。")
        return 1
    else:
        print("  结果: PASS (全部通过)")
        print()
        print("  ✓ 硬件环境就绪，可以进行下一步测试或飞行准备。")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
