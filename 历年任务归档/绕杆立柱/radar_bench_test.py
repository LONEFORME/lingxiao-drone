"""N10P 雷达台架通电测试 — 只测雷达本身，不涉及飞控/T265，不需要解锁飞机。

用法:
  DRONE_RADAR_PORT=COM9 python radar_bench_test.py      # Windows
  DRONE_RADAR_PORT=/dev/ttyUSB0 python radar_bench_test.py  # Linux/树莓派

每秒打印一次：是否收到数据、最近一圈点数、最近障碍物角度和距离。
"""
import os
import time
from Lcode.Lradar import Serial_radar
from Lcode.Logger import logger


def main():
    port = os.getenv("DRONE_RADAR_PORT", "/dev/ttyUSB0")
    baud = int(os.getenv("DRONE_RADAR_BAUD", "460800"))

    radar = Serial_radar(port, baud)
    radar.port_open()
    radar.listen_start()

    logger.info("雷达台架测试启动，端口=%s，波特率=%d，Ctrl+C 退出", port, baud)
    try:
        while True:
            time.sleep(1.0)
            alive = radar.is_alive()
            scan = radar.get_scan()
            nearest = radar.get_nearest()
            if not alive:
                logger.warning("超过1秒未收到有效帧，检查接线/波特率/供电")
                continue
            if nearest is not None:
                angle, dist_mm = nearest
                logger.info("在线 | 本圈点数=%d | 最近障碍: 角度=%d° 距离=%dmm",
                            len(scan), angle, dist_mm)
            else:
                logger.info("在线 | 本圈点数=%d | 无有效障碍点", len(scan))
    except KeyboardInterrupt:
        logger.info("用户中断")
    finally:
        radar.listen_end()
        radar.close()


if __name__ == "__main__":
    main()
