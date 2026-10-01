"""
G题空地协同智能消防系统 — 无人机侧主入口 (fire_patrol)

运行:
  python main.py

依赖:
  pyrealsense2 (或使用模拟 fallback)
  pyserial
  simple_pid
  numpy
  opencv-python (下视摄像头火情检测)

启动顺序:
  1. 创建 T265 实例
  2. 打开飞控串口 → 启动监听 + 发送线程
  3. 打开消防车广播串口(Serial_ground)
  4. 启动下视摄像头火情检测(FireVision)
  5. 创建任务 → 启动状态机
  6. 保持主线程存活
"""
import os
import time
from Lcode.global_variable import sp_side, lock
import Lcode.Lprotocol
from Lcode.fire_vision import FireVision
from Lcode.Lground import Serial_ground, start_position_heartbeat
from Lcode.Logger import logger
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

    # 2b. 下视摄像头火情检测
    fire_vision = FireVision(device=os.getenv("DRONE_FIRE_CAMERA", "/dev/video10"))  # IMX219 VSE节点，见Lcode/fire_vision.py
    fire_vision.start()  # 打不开时返回False，latest()永远全None，不阻断飞行(见设计文档)

    # 2c. 消防车广播串口——消防车是完全独立硬件(不在本仓库范围内)，这条链路
    # 打不开(比如端口存在但没有实际接线的无线模块)不应该阻断无人机自身的飞行，
    # 2026-07-16真机测试实测/dev/ttyS7节点存在但open()抛SerialException，
    # 之前没有try/except直接导致main()崩溃退出、飞机根本起不来。
    try:
        serial_ground = Serial_ground(os.getenv("DRONE_GROUND_PORT", "/dev/ttyS7"))
    except Exception as e:
        logger.error(f"消防车广播串口打不开: {e}，火情/位置广播禁用，不影响无人机自身飞行")
        serial_ground = None

    # 3. 创建任务
    mission1 = mission(re_fc, se_fc, realsense, serial_fc)
    mission1.fire_vision = fire_vision
    mission1.serial_ground = serial_ground

    # 3b. 1Hz位置心跳广播（基本要求(3)），serial_ground不可用时跳过
    if serial_ground is not None:
        start_position_heartbeat(
            serial_ground,
            get_position_cm=lambda: (
                int(realsense.get_position()[0] * 100),
                int(realsense.get_position()[1] * 100),
            ),
        )

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
        # 2026-07-17新增：之前这里只关闭了飞控串口，fire_vision的摄像头后台采集
        # 线程和serial_ground从未被显式停止/关闭——真机测试实测到，正常任务完整
        # 结束、Python解释器开始退出清理时，仍在跑的摄像头采集线程(牵涉OpenCV/V4L2
        # 的C层调用)会跟解释器的模块终结化过程竞争，触发segfault。任务已经安全
        # 降落之后才发生，不影响飞行安全，但会导致进程不能干净退出、下次启动前
        # 设备句柄状态不确定。
        try:
            fire_vision.stop()
        except Exception as e:
            logger.error(f"fire_vision.stop() 调用失败: {e}")
        if serial_ground is not None:
            try:
                serial_ground.close()
            except Exception as e:
                logger.error(f"serial_ground.close() 调用失败: {e}")
        serial_fc.send_end()
        serial_fc.close()


if __name__ == "__main__":
    main()
