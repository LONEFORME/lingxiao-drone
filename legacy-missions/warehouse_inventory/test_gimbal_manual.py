#!/usr/bin/env python3
"""
云台舵机手动测试工具
====================
直接控制云台角度（0~180°），方便手动调整激光对准。

用法（板子上）：
    cd /home/sunrise/Desktop/FJJ/warehouse_inventory
    python3 test_gimbal_manual.py

输入角度（0~180）即可旋转，输入 q 退出。
"""

import sys
import time
from pathlib import Path


class SysfsPWM:
    """sysfs PWM 驱动（50Hz 标准舵机）"""
    def __init__(self, chip=0, channel=0, frequency_hz=50, sysfs_root="/sys/class/pwm"):
        self.base = Path(sysfs_root) / f"pwmchip{chip}"
        self.path = self.base / f"pwm{channel}"
        self.channel = int(channel)
        self.period_ns = int(1_000_000_000 / frequency_hz)
        
        # 导出 PWM 通道
        if not self.path.exists():
            (self.base / "export").write_text(str(self.channel), encoding="ascii")
            time.sleep(0.2)
        
        # 初始化周期和禁用
        self._write("period", self.period_ns)
        self._write("enable", 0)

    def _write(self, name, value):
        (self.path / name).write_text(str(int(value)), encoding="ascii")

    def start(self):
        self._write("enable", 1)

    def set_duty_ns(self, duty_ns):
        self._write("duty_cycle", max(0, min(self.period_ns, int(duty_ns))))

    def stop(self):
        self._write("enable", 0)


def angle_to_pulse_ns(angle_deg, min_ns=500_000, max_ns=2_500_000):
    """将角度（0~180°）转换为舵机 PWM 脉宽（ns）"""
    if not 0 <= angle_deg <= 180:
        raise ValueError("角度必须在 0~180° 范围内")
    return int(min_ns + (angle_deg / 180.0) * (max_ns - min_ns))


def main():
    CHIP = 0
    CHANNEL = 0
    MIN_PULSE_NS = 500_000      # 0° 对应脉宽
    MAX_PULSE_NS = 2_500_000    # 180° 对应脉宽
    SETTLE_S = 0.6              # 转动后稳定等待时间

    print("=" * 60)
    print("  云台舵机手动测试工具")
    print("=" * 60)
    print(f"  PWM: chip{CHIP}/channel{CHANNEL}")
    print(f"  脉宽范围: {MIN_PULSE_NS}ns (0°) ~ {MAX_PULSE_NS}ns (180°)")
    print(f"  稳定等待: {SETTLE_S}s")
    print("=" * 60)
    print()

    pwm = None
    try:
        # 初始化 PWM
        print("[...] 初始化 PWM...")
        pwm = SysfsPWM(chip=CHIP, channel=CHANNEL)
        pwm.start()
        print("[OK] PWM 已启动")
        print()

        current_angle = None

        while True:
            print("-" * 40)
            if current_angle is not None:
                print(f"当前角度: {current_angle:.1f}°")
            print("输入目标角度 (0~180) 或 'q' 退出:")
            print("  常用: 0 (A/C面)  |  90 (中间)  |  180 (B/D面)")
            print()

            try:
                cmd = input(">>> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n[OK] 退出")
                break

            if cmd.lower() in {"q", "quit", "exit"}:
                print("[OK] 退出")
                break

            # 解析角度
            try:
                angle = float(cmd)
            except ValueError:
                print(f"[ERROR] 无效输入: {cmd}")
                continue

            if not 0 <= angle <= 180:
                print(f"[ERROR] 角度超出范围 (0~180): {angle}")
                continue

            # 转动舵机
            try:
                pulse_ns = angle_to_pulse_ns(angle, MIN_PULSE_NS, MAX_PULSE_NS)
                pwm.set_duty_ns(pulse_ns)
                print(f"[OK] 已转到 {angle:.1f}° (脉宽 {pulse_ns} ns)")
                print(f"[...] 等待舵机稳定 ({SETTLE_S}s)...")
                time.sleep(SETTLE_S)
                print("[OK] 稳定完成")
                current_angle = angle
            except Exception as e:
                print(f"[ERROR] 舵机控制失败: {e}")

            print()

    except Exception as e:
        print(f"\n[ERROR] 异常: {e}")
        import traceback
        traceback.print_exc()
        return 1

    finally:
        # 清理
        if pwm is not None:
            try:
                print("\n[...] 关闭 PWM...")
                pwm.stop()
                print("[OK] PWM 已关闭")
            except Exception as e:
                print(f"[WARN] PWM 关闭失败: {e}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
