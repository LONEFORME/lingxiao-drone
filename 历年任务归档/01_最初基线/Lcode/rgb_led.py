import wiringpi
import time

# 初始化 wiringOP（必须加！）
wiringpi.wiringPiSetup()

# RGB 引脚定义（wPi 编号）
LED_PINS = {
    'R': 6,   # 红色，物理12
    'G': 10,  # 绿色，物理16
    'B': 9,   # 蓝色，物理18
}

# 设置为输出模式
for pin in LED_PINS.values():
    wiringpi.pinMode(pin, 1)


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
        'R':   {'R': 1, 'G': 0, 'B': 0},   # 红
        'G':   {'R': 0, 'G': 1, 'B': 0},   # 绿
        'B':   {'R': 0, 'G': 0, 'B': 1},   # 蓝
        'W':   {'R': 1, 'G': 1, 'B': 1},   # 白
        'OFF': {'R': 0, 'G': 0, 'B': 0},   # 关
    }

    if color.upper() not in color_map:
        print(f"不支持的颜色: {color}，可选: {list(color_map.keys())}")
        return

    states = color_map[color.upper()]

    # 设置各引脚电平
    wiringpi.digitalWrite(LED_PINS['R'], states['R'])
    wiringpi.digitalWrite(LED_PINS['G'], states['G'])
    wiringpi.digitalWrite(LED_PINS['B'], states['B'])

    if duration > 0:
        time.sleep(duration)
        # 关闭所有
        wiringpi.digitalWrite(LED_PINS['R'], 0)
        wiringpi.digitalWrite(LED_PINS['G'], 0)
        wiringpi.digitalWrite(LED_PINS['B'], 0)


# ===== 测试代码 =====
if __name__ == "__main__":
    try:
        rgb_led('R', 1)      # 红色 1 秒
        rgb_led('G', 1)      # 绿色 1 秒
        rgb_led('B', 1)      # 蓝色 1 秒
        rgb_led('W', 2)      # 白色 2 秒
    finally:
        rgb_led('OFF', 0)    # 确保关闭
    print("RGB 测试完成！")
