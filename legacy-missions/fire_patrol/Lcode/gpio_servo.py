"""抛投机构舵机驱动。地瓜派(RDK X5)专用，sysfs PWM(/sys/class/pwm/pwmchip0/pwm0，
对应物理引脚32)，50Hz、0.5ms~2.5ms脉宽对应0°~180°——参照
`Desktop/GPIO测试/舵机示例.ipynb`验证过的接线方案(2026-07-16)。

sysfs路径在本机开发环境(Windows)不存在，写入用try/except降级为空操作，让本机
pytest能正常跑(舵机控制实际降级为空操作)，板子上真实调用时才会走真实PWM分支。

写入顺序坑(2026-07-17台架复测发现)：pwm0处于period=0的全新状态(比如刚重启后
第一次用)时，必须先写period再写enable=0，反过来会触发OSError(EINVAL)——跟
原始notebook示例的顺序(先enable再period)不同，这里改成先period后enable。

跟`gpio_led.py`一样不做阻塞式time.sleep()——Mission_GPT.py的主循环每30ms跑
一次，函数内阻塞哪怕1秒都会拖慢飞控指令节奏。set_servo_angle()只发一次角度
指令让舵机自己转，不等待转动完成，调用方自己决定要不要在之后的tick里复位。
"""
import os
import threading
import time

from Lcode.Logger import logger

PWM_CHIP = 0
PWM_CHANNEL = 0
PWM_FREQUENCY_HZ = 50
# 2026-07-17真机测试后用户指出：开机复位到0°会把已装填的沙包提前丢出去——
# 现场实测确认0°其实是舱门松开(释放)的角度，180°才是夹紧/锁定(不丢)的角度，
# 跟直觉里"0=关闭"相反，是这套抛投机构舵机的实际安装方向决定的。
SERVO_ANGLE_CLOSED = 180
SERVO_ANGLE_OPEN = 0

_PWM_BASE = f"/sys/class/pwm/pwmchip{PWM_CHIP}"
_PWM_PATH = f"{_PWM_BASE}/pwm{PWM_CHANNEL}"
_PERIOD_NS = int(1_000_000_000 / PWM_FREQUENCY_HZ)

_setup_done = False
_lock = threading.Lock()


def _write(name: str, value) -> bool:
    try:
        with open(f"{_PWM_PATH}/{name}", "w") as f:
            f.write(str(value))
        return True
    except OSError as e:
        logger.warning(f"gpio_servo: 写{name}失败({e})，GPIO不可用(非板载环境或硬件未接入)")
        return False


def _ensure_setup() -> bool:
    global _setup_done
    with _lock:
        if _setup_done:
            return True
        if not os.path.exists(_PWM_PATH):
            try:
                with open(f"{_PWM_BASE}/export", "w") as f:
                    f.write(str(PWM_CHANNEL))
                time.sleep(0.2)
            except OSError as e:
                logger.warning(f"gpio_servo: export失败({e})，GPIO不可用(非板载环境或硬件未接入)")
                return False
        # 先period后enable：period=0时写enable(哪怕写0)会EINVAL，见模块docstring。
        if not _write("period", _PERIOD_NS):
            return False
        if not _write("enable", 0):
            return False
        _setup_done = True
        return True


def _angle_to_duty_ns(angle: float) -> int:
    angle = max(0, min(180, angle))
    return 500_000 + int(angle / 180 * 2_000_000)


def set_servo_angle(angle: float) -> bool:
    """设置舵机角度并保持(0~180°)，首次调用自动初始化PWM通道。
    GPIO不可用(本机开发环境/硬件未接入)时返回False，不抛异常。"""
    if not _ensure_setup():
        return False
    duty_ns = _angle_to_duty_ns(angle)
    if not _write("duty_cycle", duty_ns):
        return False
    return _write("enable", 1)
