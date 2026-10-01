import time
import Hobot.GPIO as GPIO

# 初始化 GPIO（必须加！）
GPIO.setmode(GPIO.BCM)

# RGB 引脚定义（BCM 编号）
LED_PINS = {
    'R': 23,   # 红色
    'G': 25,   # 绿色
    'B': 24,   # 蓝色
}

# 设置为输出模式
for pin in LED_PINS.values():
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)


def rgb_led(color, duration=1):
    """
    RGB LED 控制函数

    参数：
        color  : 颜色名称，支持 'R'(红), 'G'(绿), 'B'(蓝), 'W'(白), 'OFF'(关)
        duration : 点亮持续时间（秒），默认 1 秒；设为 0 表示常亮

    示例：
        rgb_led('R')          # 红色亮 1 秒
        rgb_led('G', 2)       # 绿色亮 2 秒
        rgb_led('B', 0)       # 蓝色常亮（不自动关闭）
        rgb_led('W', 0.5)     # 白色亮 0.5 秒
        rgb_led('OFF')        # 全部关闭
    """
    # 定义颜色映射
    color_map = {
        'R':   {'R': GPIO.HIGH, 'G': GPIO.LOW, 'B': GPIO.LOW},   # 红
        'G':   {'R': GPIO.LOW, 'G': GPIO.HIGH, 'B': GPIO.LOW},   # 绿
        'B':   {'R': GPIO.LOW, 'G': GPIO.LOW, 'B': GPIO.HIGH},   # 蓝
        'W':   {'R': GPIO.HIGH, 'G': GPIO.HIGH, 'B': GPIO.HIGH}, # 白
        'OFF': {'R': GPIO.LOW, 'G': GPIO.LOW, 'B': GPIO.LOW},     # 关
    }

    if color.upper() not in color_map:
        print(f"不支持的颜色: {color}，可选: {list(color_map.keys())}")
        return

    states = color_map[color.upper()]

    # 设置各引脚电平
    GPIO.output(LED_PINS['R'], states['R'])
    GPIO.output(LED_PINS['G'], states['G'])
    GPIO.output(LED_PINS['B'], states['B'])

    if duration > 0:
        time.sleep(duration)
        # 关闭所有
        GPIO.output(LED_PINS['R'], GPIO.LOW)
        GPIO.output(LED_PINS['G'], GPIO.LOW)
        GPIO.output(LED_PINS['B'], GPIO.LOW)


# ===== 测试代码 =====
if __name__ == "__main__":
    try:
        rgb_led('R', 1)      # 红色 1 秒
        rgb_led('G', 1)      # 绿色 1 秒
        rgb_led('B', 1)      # 蓝色 1 秒
        rgb_led('W', 2)      # 白色 2 秒
    finally:
        rgb_led('OFF', 0)    # ✅ 先关闭 LED
        GPIO.cleanup()       # ✅ 再清理 GPIO
    print("RGB 测试完成！")