"""
基本飞行 — 主入口

运行:
  python main.py

依赖:
  pyrealsense2 (或使用模拟 fallback)
  pyserial
  simple_pid
  numpy

启动顺序:
  1. 创建 T265 实例
  2. 打开飞控串口 → 启动监听 + 发送线程
  3. 创建任务 → 启动状态机
  4. 保持主线程存活
"""
import os
import time
from Lcode.global_variable import sp_side, lock
import Lcode.Lprotocol
from Lcode.Logger import logger
from Lcode.Lradar import Serial_radar
from Mission_GPT import mission
from t265 import t265_class


# ======================== 变量 ========================
re_fc = [0] * 14  # [mission_stage, roll_x100, pitch_x100, yaw_x100, fusion_state, unlock_sta, integral_x, integral_y, laser_cm, of1_dx, of1_dy, of_quality, of_link_sta, of_work_sta]

# AA 02 task_sta com_x+sp com_y+sp com_z com_yaw+sp next_task sp_side CK FF
# com_z(索引5)占位为0：takeoff()会在触发task_sta前覆写为本次真正的目标高度，
# 这里绝不能填非0默认值——2026-07-06 曾因这里写死120导致一键起飞冲到1.2m（见CLAUDE.md已知问题6）
se_fc = [170, 2, 0, sp_side, sp_side, 0, sp_side, 0, sp_side, 0, 255]


# ===================== 入口 =====================
def main():
    logger.info("=" * 40)
    logger.info("basic_flight — 基本飞行控制器")
    logger.info("=" * 40)

    # 1. T265
    realsense = t265_class()

    # 2. 飞控串口
    port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
    serial_fc = Lcode.Lprotocol.Serial_fc(port, 460800)
    serial_fc.listen_start(re_fc)
    serial_fc.send_start(se_fc, realsense, vel_freq=100, cmd_freq=50)

    # 2.5 雷达(可选，DRONE_RADAR_ENABLED=1 才启用，默认关闭不影响不接雷达的测试)
    radar = None
    if os.getenv("DRONE_RADAR_ENABLED", "0") == "1":
        radar_port = os.getenv("DRONE_RADAR_PORT", "/dev/radar")  # ubuntu-pi上/dev/ttyUSB0其实是蓝牙，
                                                                    # 雷达走udev固定命名(见/etc/udev/rules.d/99-drone-serial.rules)
        radar_baud = int(os.getenv("DRONE_RADAR_BAUD", "460800"))
        radar = Serial_radar(radar_port, radar_baud)
        radar.port_open()
        radar.listen_start()
        logger.info(f"雷达避障已启用，端口={radar_port}，波特率={radar_baud}")

    # 3. 创建任务
    mission1 = mission(re_fc, se_fc, realsense, serial_fc, radar_obj=radar)

    # 4. 启动
    mission1.start()

    # 5. 保持（任务自然结束或 Ctrl+C 都会退出）
    try:
        while mission1.task_running:
            time.sleep(0.1)
        logger.info("任务已结束，程序退出")
    except KeyboardInterrupt:
        logger.info("用户中断")
        mission1.emergency()
    finally:
        serial_fc.send_end()
        serial_fc.close()


if __name__ == "__main__":
    main()
