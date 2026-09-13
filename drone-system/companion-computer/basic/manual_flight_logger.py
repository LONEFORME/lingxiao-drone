"""
手动遥控飞行 + T265 数据记录

用途:
  遥控器物理描杆飞行，Python 只负责给凌霄IMU持续提供 T265 速度参考（悬停抗漂移），
  不发送任何指令帧(AA 02)，不涉及 task_sta/com_x/y/z/yaw，不跑 Mission_GPT 状态机。
  飞控收不到指令帧，received_data.task_sta 保持默认值0，"定位任务"状态机不会被
  Python 触发，完全由遥控器 CH_7/CH_8 物理开关控制飞行模式。

运行:
  python manual_flight_logger.py

依赖: 同 main.py (pyrealsense2/pyserial/numpy)
"""
import json
import os
import time

import Lcode.Lprotocol
from Lcode.Logger import logger
from t265 import t265_class

LOG_INTERVAL = 0.05  # 20Hz，跟自动飞行日志频率一致


def main():
    logger.info("=" * 40)
    logger.info("manual_flight_logger — 手动遥控飞行数据记录")
    logger.info("=" * 40)

    re_fc = [0] * 14

    realsense = t265_class()
    realsense.start()

    port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
    serial_fc = Lcode.Lprotocol.Serial_fc(port, 460800)
    serial_fc.listen_start(re_fc)
    serial_fc.send_start(t265_obj=realsense, vel_freq=100)  # 不传 comlist，不发指令帧

    path = os.path.dirname(os.path.realpath(__file__))
    log_file = open(path + "/flight_data_manual.jsonl", "a")
    log_file.write(json.dumps({"event": "manual_flight_start"}) + "\n")
    log_file.flush()

    last_log_time = 0.0

    try:
        while True:
            pos = realsense.get_position()
            tv = realsense.get_velocity()

            of1_dx = re_fc[9] if len(re_fc) > 9 else 0
            of1_dy = re_fc[10] if len(re_fc) > 10 else 0
            roll_deg = re_fc[1] / 100.0 if len(re_fc) > 1 else 0.0
            pitch_deg = re_fc[2] / 100.0 if len(re_fc) > 2 else 0.0
            of_quality = re_fc[11] if len(re_fc) > 11 else 0
            of_link_sta = re_fc[12] if len(re_fc) > 12 else 0
            of_work_sta = re_fc[13] if len(re_fc) > 13 else 0

            now = time.time()
            if now - last_log_time >= LOG_INTERVAL:
                try:
                    log_file.write(json.dumps({
                        "t": round(now, 3),
                        "pos": [round(pos[0], 4), round(pos[1], 4), round(pos[2], 4)],
                        "t265_vel": [round(tv[0], 4), round(tv[1], 4)],
                        "of1_vel_cms": [of1_dx, of1_dy],
                        "roll_pitch": [round(roll_deg, 2), round(pitch_deg, 2)],
                        "of_status": [of_quality, of_link_sta, of_work_sta],
                    }) + "\n")
                    log_file.flush()
                except Exception:
                    pass
                last_log_time = now

                print(
                    f"\rpos=({pos[0]:+.3f},{pos[1]:+.3f},{pos[2]:+.3f}) "
                    f"| t265v=({tv[0]:+.2f},{tv[1]:+.2f}) "
                    f"| of1=({of1_dx:+d},{of1_dy:+d}) "
                    f"| att=({roll_deg:+.1f},{pitch_deg:+.1f}) "
                    f"| of_status=({of_quality},{of_link_sta},{of_work_sta})",
                    end="", flush=True
                )

            time.sleep(0.01)
    except KeyboardInterrupt:
        logger.info("用户中断")
    finally:
        log_file.close()
        realsense.stop()
        serial_fc.send_end()
        serial_fc.close()
        logger.info("manual_flight_logger 已退出")


if __name__ == "__main__":
    main()
