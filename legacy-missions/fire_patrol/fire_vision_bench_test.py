"""火情检测台架测试 — 手持/轻晃摄像头，不涉及飞控/起飞。

用法:
  python fire_vision_bench_test.py [--duration 30] [--device /dev/video10]

跑起来后把红色光源（越接近赛题要求的LED+喇叭形遮光罩越好）放进摄像头视野，
持续观察终端输出的检测结果；测试过程中轻轻晃动/小幅移动摄像头，
模拟飞行时的振动/微小位移，看识别是否稳定（不丢失、不剧烈跳变）。

见 docs/superpowers/specs/2026-07-16-fire-patrol-design.md "APPROACH"一节
关于HSV阈值/面积阈值需要现场标定的说明。
"""
import argparse
import time

from Lcode.fire_vision import FireVision, IMX219_DEVICE


def parse_arguments():
    parser = argparse.ArgumentParser(description="fire_patrol 火情检测台架测试")
    parser.add_argument("--duration", type=float, default=30.0, help="测试时长(秒)")
    parser.add_argument("--device", default=IMX219_DEVICE)
    parser.add_argument("--print-interval", type=float, default=0.3, help="打印间隔(秒)")
    return parser.parse_args()


def main():
    args = parse_arguments()

    fv = FireVision(device=args.device)
    if not fv.start():
        print(f"摄像头打不开({args.device})，测试无法进行")
        return

    print(f"火情检测台架测试开始，持续{args.duration:.0f}秒，设备={args.device}")
    print("请把红色光源放进视野，测试过程中轻晃/小幅移动摄像头模拟飞行振动")
    print("-" * 60)

    t_start = time.time()
    last_print = 0.0
    detected_count = 0
    total_ticks = 0
    last_state = None  # None=未检测到, True=检测到，用于统计状态切换次数(抖动指标)
    switch_count = 0

    try:
        while time.time() - t_start < args.duration:
            latest = fv.latest()
            dx_px, dy_px = latest.get("dx_px"), latest.get("dy_px")
            detected = dx_px is not None
            total_ticks += 1
            if detected:
                detected_count += 1

            if last_state is not None and detected != last_state:
                switch_count += 1
            last_state = detected

            now = time.time()
            if now - last_print >= args.print_interval:
                elapsed = now - t_start
                if detected:
                    print(f"[{elapsed:5.1f}s] 检测到火源 dx={dx_px:+7.1f}px dy={dy_px:+7.1f}px")
                else:
                    print(f"[{elapsed:5.1f}s] 未检测到")
                last_print = now

            time.sleep(0.05)
    except KeyboardInterrupt:
        print("用户中断")
    finally:
        fv.stop()

    print("-" * 60)
    detect_ratio = detected_count / total_ticks if total_ticks else 0.0
    print(f"测试结束：共{total_ticks}次采样，检测到火源占比{detect_ratio * 100:.1f}%，"
          f"检测状态切换{switch_count}次（切换次数越多说明识别越不稳定/抖动越明显）")


if __name__ == "__main__":
    main()
