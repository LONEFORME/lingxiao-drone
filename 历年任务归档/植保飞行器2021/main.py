"""
植保飞行器 (G题) — 主入口

用法:
    python main.py                    # 正常飞行（等待按键后启动）
    python main.py --dry-plan         # 预览航线，不解锁飞控
    python main.py --dry-plan --phase scout
"""
from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="2021 植保飞行器")
    p.add_argument("--dry-plan", action="store_true",
                    help="预览航线，不解锁飞控")
    p.add_argument("--config", type=Path, default=_HERE / "plant_config.json",
                    help="场地配置文件路径")
    return p


def dry_plan() -> None:
    """预览模式：读取 router.txt 或 plant_config 生成航线并打印。"""
    here = Path(__file__).resolve().parent
    router = here / "router.txt"

    if router.exists():
        # 优先使用 router.txt（手动标定路径）
        waypoints: list[tuple[float, float, float]] = []
        with open(router) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#"):
                    parts = line.split(",")
                    if len(parts) >= 3:
                        waypoints.append((
                            float(parts[0]), float(parts[1]), float(parts[2]),
                        ))
        source = "router.txt"
    else:
        # 回退：从 plant_config.json 自动生成
        from Lcode.grid_map import GridMap
        from Lcode.coverage_planner import plan_coverage_path
        gm = GridMap(here / "plant_config.json")
        waypoints = plan_coverage_path(gm)
        source = "plant_config.json (自动生成)"

    if not waypoints:
        print("⚠ 没有航点数据")
        return

    print(f"\n{'='*50}")
    print(f"植保航线预览 — {source}")
    print(f"航点总数: {len(waypoints)}")
    print(f"{'='*50}\n")

    for i, wp in enumerate(waypoints):
        is_home = (i == 0 or i == len(waypoints) - 1)
        is_land = is_home and wp[2] < 0.5
        label = "LAND" if is_land else ("HOME" if is_home else f"WP{i}")
        print(f"  {label:>6}: ({wp[0]:+.2f}, {wp[1]:+.2f}, {wp[2]:.2f})")

    print(f"\n航线总长: {_total_length(waypoints):.1f}m")
    print("=" * 50)


def _total_length(waypoints: list[tuple[float, float, float]]) -> float:
    total = 0.0
    for i in range(1, len(waypoints)):
        dx = waypoints[i][0] - waypoints[i - 1][0]
        dy = waypoints[i][1] - waypoints[i - 1][1]
        total += (dx * dx + dy * dy) ** 0.5
    return total


def _wait_for_start_button() -> bool:
    """只初始化GPIO，绿灯等待用户完成T265拔插后按键。"""
    try:
        from Lcode.gpio_button import GpioButton
        from Lcode.gpio_led import set_rgb_led
    except Exception as e:
        from Lcode.Logger import logger
        logger.error(f"一键起飞GPIO模块加载失败: {e}")
        return False

    from Lcode.Logger import logger

    button = GpioButton()
    led_is_off = True
    try:
        if not button.start():
            logger.error("一键起飞按钮初始化失败")
            return False
        led_is_off = False
        if not set_rgb_led('G'):
            logger.error("一键起飞绿灯点亮失败")
            return False

        logger.info("绿灯常亮：请完成T265拔插，然后按下一键起飞按钮")
        while not button.was_pressed():
            time.sleep(0.05)

        logger.info("一键起飞按钮已按下")
        if not set_rgb_led('OFF'):
            logger.error("按键确认后关闭绿灯失败")
            return False
        led_is_off = True
        return True
    except KeyboardInterrupt:
        logger.info("等待一键起飞按钮时收到用户中断")
        raise
    except Exception as e:
        logger.error(f"一键起飞按钮等待失败: {e}")
        return False
    finally:
        if not led_is_off:
            try:
                set_rgb_led('OFF')
            except Exception:
                pass
        try:
            button.stop()
        except Exception:
            pass


def main():
    parser = build_parser()
    args = parser.parse_args()

    if args.dry_plan:
        dry_plan()
        return

    # ── 正常飞行模式 ──────────────────────────────────
    import Lcode.Lprotocol
    from Lcode.Logger import logger
    from Lcode.global_variable import sp_side
    from Mission_GPT import mission
    from t265 import t265_class

    # 植保模式：通过环境变量传递配置文件路径（用于动态重规划场景）
    # 当前使用 router.txt 静态航线，不设置 PLANT_CONFIG
    # 当需要动态避障时可设置: os.environ["PLANT_CONFIG"] = str(args.config)

    DRY_RUN = os.getenv("DRONE_DRY_RUN", "0") == "1"
    if not DRY_RUN:
        if not _wait_for_start_button():
            logger.error("一键起飞按钮等待失败，退出")
            return

    # 初始化 T265
    realsense1 = t265_class()
    realsense_ok = realsense1.start()
    if realsense_ok:
        realsense1.autoset()

    # 打开飞控串口
    port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
    logger.info(f"打开飞控串口: {port}")
    try:
        serial_fc = Lcode.Lprotocol.Serial_fc(port, 460800)
    except Exception as e:
        logger.error(f"飞控串口打开失败: {e}")
        if realsense_ok:
            realsense1.stop()
        return

    re_fc = [0] * 14
    se_fc = [0] * 11

    serial_fc.listen_start(re_fc)
    serial_fc.send_start(se_fc, realsense1 if realsense_ok else None)

    # 初始化摄像头 + 颜色识别（不影响飞行，失败时静默降级）
    video_src = None
    color_det = None
    try:
        from Lcode.video_source import create_video_source, load_video_config
        from Lcode.color_detector import ColorDetector
        video_config = load_video_config(args.config)
        if video_config.enabled:
            video_src = create_video_source(video_config)
            video_src.start()
        color_det = ColorDetector()
    except Exception as e:
        logger.info(f"颜色识别未启用（{e}），不影响飞行")

    # 创建任务
    mission1 = mission(re_fc, se_fc,
                       realsense_obj=realsense1 if realsense_ok else None,
                       serial_fc_ref=serial_fc,
                       video_source=video_src,
                       color_detector=color_det)

    # 红灯警示 5 秒
    logger.info("红灯 5 秒警示…")
    try:
        from Lcode.gpio_led import set_rgb_led
        set_rgb_led('R')
        time.sleep(5)
        set_rgb_led('OFF')
    except Exception:
        logger.warning("GPIO LED 不可用，跳过红灯警示")
    mission1.start()

    try:
        while mission1.task_running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("收到用户中断")
    finally:
        mission1.emergency()
        mission1.stop_all()
        serial_fc.stop()
        if realsense_ok:
            realsense1.stop()
        logger.info("程序退出")


if __name__ == "__main__":
    main()
