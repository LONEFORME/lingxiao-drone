"""
基本飞行 — 主入口

运行:
  python main.py

启动顺序:
  1. 绿灯常亮等待 BCM17 一键起飞按钮
  2. 按键后创建 T265 实例
  3. 打开飞控串口 → 启动监听 + 发送线程
  4. 创建任务 → T265检查 → 红灯5秒 → 启动状态机
  5. 保持主线程存活
"""
import os
import time

import Lcode.Lprotocol
from Lcode.global_variable import sp_side
from Lcode.Logger import logger
from Mission_GPT import mission
from t265 import t265_class


START_BUTTON_POLL_S = 0.05


def wait_for_start_button():
    """只初始化GPIO，绿灯等待用户完成T265拔插后按键。"""
    try:
        from Lcode.gpio_button import GpioButton
        from Lcode.gpio_led import set_rgb_led
    except Exception as e:
        logger.error(f"一键起飞GPIO模块加载失败: {e}")
        return False

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
            time.sleep(START_BUTTON_POLL_S)

        logger.info("一键起飞按钮已按下，开始初始化T265和飞控串口")
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
            except Exception as e:
                logger.error(f"一键起飞等待结束时关灯失败: {e}")
        try:
            button.stop()
        except Exception as e:
            logger.error(f"一键起飞按钮资源释放失败: {e}")


def main():
    logger.info("=" * 40)
    logger.info("basic_flight — 基本飞行控制器")
    logger.info("=" * 40)

    # 先等待用户完成T265拔插并按键；此时不创建T265、不打开飞控串口。
    if not wait_for_start_button():
        logger.error("一键起飞门禁失败，程序退出；飞控不会解锁")
        return

    # 按键通过后才创建通信缓冲区和硬件对象。
    re_fc = [0] * 14
    # com_z(索引5)必须保持0；takeoff()会在触发task_sta前写入本次目标高度。
    se_fc = [170, 2, 0, sp_side, sp_side, 0, sp_side, 0, sp_side, 0, 255]
    realsense = t265_class()

    serial_fc = None
    mission1 = None
    try:
        port = os.getenv("DRONE_FC_PORT", "/dev/ttyS6")
        serial_fc = Lcode.Lprotocol.Serial_fc(port, 460800)
        serial_fc.listen_start(re_fc)
        serial_fc.send_start(se_fc, realsense, vel_freq=100, cmd_freq=50)

        mission1 = mission(re_fc, se_fc, realsense, serial_fc)
        mission1.start()
        while mission1.task_running:
            time.sleep(0.1)
        logger.info("任务已结束，程序退出")
    except KeyboardInterrupt:
        logger.info("用户中断")
        if mission1 is not None:
            mission1.emergency()
            if not mission1.task_running:
                mission1.stop_all()
    finally:
        if serial_fc is not None:
            serial_fc.send_end()
            serial_fc.close()


if __name__ == "__main__":
    main()
