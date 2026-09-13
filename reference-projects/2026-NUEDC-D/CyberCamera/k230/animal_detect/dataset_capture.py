# K230离线数据集自动采集脚本
# 功能: 开机自动按间隔拍照, 按键切换类别, 图片存至TF卡
# 显示: HDMI 1920x1080
#
# 使用方法:
#   1. 修改下方 CLASS_NAMES 为你的类别名
#   2. 将本文件重命名为 main.py 放入TF卡根目录
#   3. 上电即自动运行

import time
import os

from media.sensor import Sensor, CAM_CHN_ID_0
from media.display import Display
from media.media import MediaManager

from machine import Pin
from machine import FPIOA

# ======================== 可配置参数 ========================

CLASS_NAMES = [
    "class_01",
    "class_02",
    "class_03",
    "class_04",
    "class_05",
]

AUTO_INTERVAL = 1000      # 自动拍摄间隔 (毫秒)
SWITCH_PAUSE  = 10000       # 切换类别后暂停时间 (毫秒)
JPEG_QUALITY  = 95         # JPEG压缩质量 (1-100)
DEBOUNCE_MS   = 200        # 按键消抖 (毫秒)

RESOLUTION_W  = 1920       # 拍摄宽度
RESOLUTION_H  = 1080       # 拍摄高度

BASE_FOLDER   = "/data/dataset"  # TF卡存储根目录

# ======================== 硬件初始化 ========================

fpioa = FPIOA()
fpioa.set_function(62, FPIOA.GPIO62)
fpioa.set_function(20, FPIOA.GPIO20)
fpioa.set_function(63, FPIOA.GPIO63)
fpioa.set_function(53, FPIOA.GPIO53)

LED_R = Pin(62, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
LED_G = Pin(20, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
LED_B = Pin(63, Pin.OUT, pull=Pin.PULL_NONE, drive=7)
LED_R.high()
LED_G.high()
LED_B.high()

BUTTON = Pin(53, Pin.IN, Pin.PULL_DOWN)


def led_pulse(led, ms=50):
    led.low()
    time.sleep_ms(ms)
    led.high()


def led_pulse_n(led, count, ms=50):
    for _ in range(count):
        led.low()
        time.sleep_ms(ms)
        led.high()
        if count > 1:
            time.sleep_ms(ms)


# ======================== 文件管理 ========================

def ensure_dir(path):
    try:
        os.stat(path)
    except OSError:
        os.mkdir(path)


def scan_class_count(class_name):
    folder = "%s/%s" % (BASE_FOLDER, class_name)
    try:
        files = os.listdir(folder)
    except OSError:
        return 0
    nums = []
    for f in files:
        if f.endswith(".jpg"):
            try:
                nums.append(int(f.split(".")[0]))
            except ValueError:
                pass
    return max(nums) if nums else 0


def save_jpg(img, class_name, index):
    folder = "%s/%s" % (BASE_FOLDER, class_name)
    path = "%s/%05d.jpg" % (folder, index)
    data = img.compress(quality=JPEG_QUALITY)
    with open(path, "wb") as f:
        f.write(data)


# ======================== 启动初始化 ========================

ensure_dir(BASE_FOLDER)
for cls in CLASS_NAMES:
    ensure_dir("%s/%s" % (BASE_FOLDER, cls))

class_counts = [scan_class_count(cls) for cls in CLASS_NAMES]
class_index = 0

print("[dataset] base: %s" % BASE_FOLDER)
for i, cls in enumerate(CLASS_NAMES):
    print("[dataset]   %s: %d images" % (cls, class_counts[i]))
print("[dataset] start class: %s" % CLASS_NAMES[class_index])

def _draw_text(img, x, y, size, text, color):
    outline = (0, 0, 0)
    for dx in (-2, 0, 2):
        for dy in (-2, 0, 2):
            if dx == 0 and dy == 0:
                continue
            img.draw_string_advanced(x + dx, y + dy, size, text, color=outline)
    img.draw_string_advanced(x, y, size, text, color=color)


# ======================== 摄像头与显示 ========================
sensor = None

try:
    sensor = Sensor(id=1)
    sensor.reset()
    sensor.set_framesize(width=RESOLUTION_W, height=RESOLUTION_H, chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.RGB565, chn=CAM_CHN_ID_0)

    Display.init(Display.LT9611, width=RESOLUTION_W, height=RESOLUTION_H, to_ide=True)

    MediaManager.init()
    sensor.run()

    # ======================== 主循环 ========================

    running = False         # 是否已启动自动采集
    last_capture = time.ticks_ms()
    pause_until = 0
    btn_last = 0
    last_btn_time = 0

    while True:
        os.exitpoint()
        now = time.ticks_ms()

        img = sensor.snapshot(chn=CAM_CHN_ID_0)

        # ---- 按键 ----
        btn = BUTTON.value()
        if btn == 1 and btn_last == 0:
            if time.ticks_diff(now, last_btn_time) >= DEBOUNCE_MS:
                if not running:
                    # 首次按下：启动自动采集
                    running = True
                    last_capture = now
                    print("[dataset] started, class: %s" % CLASS_NAMES[class_index])
                    led_pulse_n(LED_G, 3)
                else:
                    # 已启动：切换类别
                    class_index = (class_index + 1) % len(CLASS_NAMES)
                    cls = CLASS_NAMES[class_index]
                    print("[dataset] -> %s (%d images)" % (cls, class_counts[class_index]))
                    pause_until = now + SWITCH_PAUSE
                    last_capture = now
                    led_pulse_n(LED_B, 2)
                last_btn_time = now
        btn_last = btn

        # ---- 自动拍照逻辑 ----
        cls = CLASS_NAMES[class_index]
        cnt = class_counts[class_index]
        in_pause = pause_until > now

        if running and not in_pause:
            elapsed = time.ticks_diff(now, last_capture)
            if elapsed >= AUTO_INTERVAL:
                cnt += 1
                class_counts[class_index] = cnt
                save_jpg(img, cls, cnt)
                print("[dataset] save: %s/%05d.jpg" % (cls, cnt))
                last_capture = now
                led_pulse(LED_G, 30)

        # ---- LCD 叠加信息 ----
        total = sum(class_counts)

        if not running:
            status = "Press button to start"
            sc = (0, 255, 128)
            cls_info = "%s [%d]" % (cls, cnt)
        elif in_pause:
            remain_sec = time.ticks_diff(pause_until, now) / 1000.0
            status = "PREPARE %.1fs" % max(0, remain_sec)
            sc = (255, 120, 0)
            cls_info = "%s [%d]" % (cls, cnt)
        else:
            remain_sec = (AUTO_INTERVAL - time.ticks_diff(now, last_capture)) / 1000.0
            status = "AUTO  next:%.1fs" % max(0, remain_sec)
            sc = (0, 255, 0)
            cls_info = "%s [%d]" % (cls, cnt)

        # 顶部: 类别名 + 数量
        _draw_text(img, 20, 14, 44, cls_info, (255, 255, 255))

        # 顶部右侧: 状态 / 倒计时
        _draw_text(img, RESOLUTION_W - 700, 18, 32, status, sc)

        # 底部左侧: 总数量
        _draw_text(img, 20, RESOLUTION_H - 56, 32, "Total: %d" % total, (255, 255, 0))

        # 底部右侧: 当前类别/总类别
        _draw_text(img, RESOLUTION_W - 280, RESOLUTION_H - 56, 28,
                   "%d/%d" % (class_index + 1, len(CLASS_NAMES)), (180, 180, 180))

        Display.show_image(img)

except KeyboardInterrupt:
    print("[dataset] user stopped")
except BaseException as e:
    print("[dataset] error: %s" % e)
    led_pulse_n(LED_R, 5, 200)
    raise
finally:
    if sensor is not None and isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
    LED_R.high()
    LED_G.high()
    LED_B.high()
    print("[dataset] cleanup done")
