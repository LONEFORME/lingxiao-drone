"""警示LED / 抛投机构接口。2026-07-16警示LED硬件已确认就绪(参照
Desktop/GPIO测试/LED测试.ipynb)，warn_led()改为调用Lcode/gpio_led.py真实GPIO；
2026-07-17抛投机构(舵机)硬件也已确认就绪(参照Desktop/GPIO测试/舵机示例.ipynb)，
drop_bag()改为调用Lcode/gpio_servo.py真实PWM舵机。两者调用方(Mission_GPT.py)
都不用改动，接口签名不变。"""
import time

from Lcode.Logger import logger
from Lcode.gpio_led import set_rgb_led
from Lcode.gpio_servo import set_servo_angle, SERVO_ANGLE_OPEN, SERVO_ANGLE_CLOSED

# 抛投舵机在释放角度停留的时长：给舵机转到位+沙包借重力脱离留时间，太短可能
# 舵机还没转到位就复位回去，沙包卡在半松开状态。
DROP_HOLD_S = 1.0


def warn_led() -> bool:
    """点亮警示LED(常亮红色)，示警识别到火情。GPIO不可用(本机开发环境/硬件未接)
    时降级为仅打日志，不影响调用方(仍返回True，表示"示警动作已发起")。"""
    if set_rgb_led('R'):
        logger.info("warn_led(): 警示LED已点亮(红色)")
    else:
        logger.info("[占位] warn_led(): GPIO不可用，仅记录日志（硬件未接入或非板载环境）")
    return True


def drop_bag() -> bool:
    """触发抛投机构舵机转到释放角度打开舱门放出灭火包，停留DROP_HOLD_S秒后
    立即复位回锁定角度，不能让舱门一直开着——2026-07-17真机测试后用户指出：
    如果抛投后不复位，下次main.py启动时的开机复位步骤(见main.py)会把新装填
    的沙包在起飞前就提前松开丢出去。DROP_HOLD_S期间做阻塞式time.sleep()：
    调用方(Mission_GPT.py的_do_hover_drop())此时已经闭环稳定悬停3秒，se_fc
    在这短暂停留期间不会变化，飞控继续按最后一次收到的指令飞行，阻塞约1秒
    不影响飞行稳定性(不同于navigate()那种每30ms必须响应的场景)。GPIO不可用
    (本机开发环境/硬件未接)时降级为仅打日志，不影响调用方(仍返回True，表示
    "抛投动作已发起")。"""
    if set_servo_angle(SERVO_ANGLE_OPEN):
        logger.info("drop_bag(): 抛投舵机已转到释放角度，松开灭火包")
        time.sleep(DROP_HOLD_S)
        set_servo_angle(SERVO_ANGLE_CLOSED)
        logger.info("drop_bag(): 抛投舵机已复位到锁定角度")
    else:
        logger.info("[占位] drop_bag(): GPIO不可用，仅记录日志（硬件未接入或非板载环境）")
    return True
