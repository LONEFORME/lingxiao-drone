"""yaw_sign 标定工具 — 台架测试，不解锁飞控、不用电机，只连雷达+T265。

背景：`PoleTracker`/`body_to_world_xy()` 的世界系坐标转换需要一个 yaw_sign(+1或-1)，
因为 t265.py 内部经过多层轴重映射+取反+欧拉角提取，get_orientation()[2] 的符号约定
不是标准数学CCW正角度，具体该用哪个符号一直没有标定（见
docs/superpowers/specs/2026-07-08-yaw-sign-calibration-tool-design.md）。

操作方法：
  1. 把雷达和T265刚性固定在一起(模拟真实装机状态)，雷达对准一个固定目标(比如台架
     测试用的杆子/墙角)
  2. 运行本脚本，记下初始的 sign=+1 和 sign=-1 两组世界坐标
  3. 原地转动机体(不要平移)，持续观察两组坐标——基本不变的那组对应的符号就是正确的
     yaw_sign
  4. 记下结果，手动改 Mission_GPT.py 里的 POLE_YAW_SIGN 常量

用法:
  DRONE_RADAR_PORT=COM9 python calibrate_yaw_sign.py      # Windows
  DRONE_RADAR_PORT=/dev/ttyUSB0 python calibrate_yaw_sign.py  # Linux/树莓派
"""
import math
import os
import time

from Lcode.Lradar import Serial_radar, radar_angle_to_body_xy, body_to_world_xy
from Lcode.Logger import logger
from t265 import t265_class


def main():
    port = os.getenv("DRONE_RADAR_PORT", "/dev/ttyUSB0")
    baud = int(os.getenv("DRONE_RADAR_BAUD", "460800"))

    radar = Serial_radar(port, baud)
    radar.port_open()
    radar.listen_start()

    realsense = t265_class()
    realsense.start()
    realsense.autoset()

    logger.info("yaw_sign 标定工具启动，雷达端口=%s，波特率=%d，Ctrl+C 退出", port, baud)
    logger.info("请把雷达对准固定目标，原地转动机体，观察哪组坐标基本不变")

    try:
        while True:
            time.sleep(0.2)

            nearest = radar.get_nearest()
            if nearest is None:
                print("\r未检测到雷达目标...", end="", flush=True)
                continue

            angle, dist_mm = nearest
            bx, by = radar_angle_to_body_xy(angle, dist_mm)

            x, y, _ = realsense.get_position()
            yaw = realsense.get_orientation()[2]

            wx_pos, wy_pos = body_to_world_xy(x, y, yaw, bx, by, yaw_sign=1)
            wx_neg, wy_neg = body_to_world_xy(x, y, yaw, bx, by, yaw_sign=-1)

            print(
                f"\r机体位姿: x={x:+.3f} y={y:+.3f} yaw={math.degrees(yaw):+.1f}° | "
                f"sign=+1: ({wx_pos:+.3f},{wy_pos:+.3f}) | "
                f"sign=-1: ({wx_neg:+.3f},{wy_neg:+.3f})",
                end="", flush=True,
            )
    except KeyboardInterrupt:
        logger.info("用户中断")
    finally:
        radar.listen_end()
        radar.close()
        realsense.stop()


if __name__ == "__main__":
    main()
