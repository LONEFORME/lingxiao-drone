"""
激光高度实时监视器

用途:
  只监听飞控下行遥测，实时显示光流模块回传的激光高度读数（re_fc[8]，单位cm）。
  不需要T265，不发送任何指令帧，纯只读诊断工具。用于对照已知参考高度，
  验证激光高度传感器是否存在系统性偏差（见 CLAUDE.md 已知问题6候选原因④）。

运行:
  python laser_height_monitor.py
"""
import os
import time

import Lcode.Lprotocol
from Lcode.Logger import logger


def main():
    logger.info("=" * 40)
    logger.info("laser_height_monitor — 激光高度实时监视器")
    logger.info("=" * 40)

    re_fc = [0] * 14

    port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
    serial_fc = Lcode.Lprotocol.Serial_fc(port, 460800)
    serial_fc.listen_start(re_fc)

    try:
        while True:
            laser_cm = re_fc[8] if len(re_fc) > 8 else 0
            print(f"\r激光高度: {laser_cm:>6d} cm  ({laser_cm / 100.0:.2f} m)", end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        logger.info("用户中断")
    finally:
        serial_fc.listen_end()
        serial_fc.close()
        logger.info("laser_height_monitor 已退出")


if __name__ == "__main__":
    main()
