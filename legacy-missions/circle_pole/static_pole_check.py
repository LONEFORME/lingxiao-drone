"""静态杆子检测确认 — 机体静止不解锁，只跑雷达+T265+PoleTracker，验证能否稳定confirm出
已知位置的杆子。零风险，不涉及电机/解锁。

用法:
  DRONE_RADAR_PORT=/dev/ttyACM0 python static_pole_check.py [采集秒数，默认30]
"""
import math
import os
import sys
import time
from Lcode.Lradar import Serial_radar, PoleTracker
from Lcode.Logger import logger
from t265 import t265_class


def main():
    duration_s = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    port = os.getenv("DRONE_RADAR_PORT", "/dev/ttyACM0")
    baud = int(os.getenv("DRONE_RADAR_BAUD", "460800"))

    radar = Serial_radar(port, baud)
    radar.port_open()
    radar.listen_start()

    realsense = t265_class()
    realsense.start()

    logger.info("等待T265追踪置信度达标...")
    t0 = time.time()
    while time.time() - t0 < 8.0:
        if realsense.get_tracking_confidence() >= 2:
            break
        time.sleep(0.1)
    logger.info("T265追踪置信度=%d，开始静态检测，时长%.0f秒", realsense.get_tracking_confidence(), duration_s)

    tracker = PoleTracker()
    start = time.time()
    while time.time() - start < duration_s:
        x, y, z = realsense.get_position()
        _, _, yaw = realsense.get_orientation()
        tracker.update(radar, x, y, yaw)
        poles = tracker.confirmed_poles()
        if poles:
            for p in poles:
                dist = ((p['x'] - x) ** 2 + (p['y'] - y) ** 2) ** 0.5
                logger.info("确认杆子: world=(%.3f,%.3f) hits=%d 距当前位置=%.2fm  (机体pos=(%.3f,%.3f) yaw=%.1f°)",
                            p['x'], p['y'], p['hits'], dist, x, y, math.degrees(yaw))
        else:
            logger.info("未确认杆子 (机体pos=(%.3f,%.3f) yaw=%.1f°)", x, y, math.degrees(yaw))
        time.sleep(0.5)

    realsense.stop()
    logger.info("静态检测结束")


if __name__ == "__main__":
    main()
