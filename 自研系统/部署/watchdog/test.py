#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
按键激活T265 - 最终版
"""

import time
import sys
import Hobot.GPIO as GPIO
from i2cdev import I2C

# ============ 配置 ============
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

BUTTON_PIN = 17

LED_PINS = {
    'R': 23, 'G': 25, 'B': 24,
}

GPIO.setup(BUTTON_PIN, GPIO.IN)
for pin in LED_PINS.values():
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

I2C_BUS = 5
OLED_ADDR = 0x3C

# 全局标志
cleaned_up = False

# ============ OLED ============
def oled_write_cmd(i2c, cmd):
    i2c.write(bytes([0x00, cmd]))
def oled_write_data(i2c, data):
    i2c.write(bytes([0x40, data]))
def oled_set_pos(i2c, page, col):
    oled_write_cmd(i2c, 0xB0 + page)
    oled_write_cmd(i2c, 0x00 + (col & 0x0F))
    oled_write_cmd(i2c, 0x10 + (col >> 4))
def oled_init(i2c):
    cmds = [0xAE,0x20,0x00,0xB0,0xC8,0x00,0x10,0x40,0x81,0xCF,0xA1,0xA6,0xA8,0x3F,0xA4,0xD3,0x00,0xD5,0xF0,0xD9,0x22,0xDA,0x12,0xDB,0x20,0x8D,0x14,0xAF]
    for cmd in cmds:
        oled_write_cmd(i2c, cmd)
        time.sleep(0.001)
def oled_clear(i2c):
    for page in range(8):
        oled_set_pos(i2c, page, 0)
        for col in range(128):
            oled_write_data(i2c, 0x00)

FONT = {
    'A':[0x7E,0x11,0x11,0x11,0x7E],'B':[0x7F,0x49,0x49,0x49,0x36],
    'C':[0x3E,0x41,0x41,0x41,0x22],'D':[0x7F,0x41,0x41,0x22,0x1C],
    'E':[0x7F,0x49,0x49,0x49,0x41],'F':[0x7F,0x09,0x09,0x09,0x01],
    'G':[0x3E,0x41,0x49,0x49,0x7A],'H':[0x7F,0x08,0x08,0x08,0x7F],
    'I':[0x00,0x41,0x7F,0x41,0x00],'K':[0x7F,0x08,0x14,0x22,0x41],
    'L':[0x7F,0x40,0x40,0x40,0x40],'M':[0x7F,0x02,0x0C,0x02,0x7F],
    'N':[0x7F,0x04,0x08,0x10,0x7F],'O':[0x3E,0x41,0x41,0x41,0x3E],
    'P':[0x7F,0x09,0x09,0x09,0x06],'R':[0x7F,0x09,0x19,0x29,0x46],
    'S':[0x46,0x49,0x49,0x49,0x31],'T':[0x01,0x01,0x7F,0x01,0x01],
    'U':[0x3F,0x40,0x40,0x40,0x3F],'V':[0x3F,0x40,0x40,0x40,0x3F],
    'W':[0x3F,0x40,0x38,0x40,0x3F],'Y':[0x07,0x08,0x70,0x08,0x07],
    '0':[0x3E,0x51,0x49,0x45,0x3E],'1':[0x00,0x42,0x7F,0x40,0x00],
    '2':[0x42,0x61,0x51,0x49,0x46],'3':[0x21,0x41,0x45,0x4B,0x31],
    '4':[0x18,0x14,0x12,0x7F,0x10],'5':[0x27,0x45,0x45,0x45,0x39],
    '6':[0x3C,0x4A,0x49,0x49,0x30],'7':[0x01,0x71,0x09,0x05,0x03],
    '8':[0x36,0x49,0x49,0x49,0x36],'9':[0x06,0x49,0x49,0x29,0x1E],
    ' ':[0x00,0x00,0x00,0x00,0x00],'.':[0x00,0x60,0x60,0x00,0x00],
    '!':[0x00,0x5F,0x00,0x00,0x00],'-':[0x08,0x08,0x08,0x08,0x08],
    '=':[0x14,0x14,0x14,0x14,0x14],
}

def draw_text(i2c, text, page, col=0):
    for char in text:
        if col >= 128: break
        if char in FONT:
            oled_set_pos(i2c, page, col)
            for i in range(5):
                oled_write_data(i2c, FONT[char][i])
            oled_write_data(i2c, 0x00)
        col += 6

# ============ RGB LED ============
def rgb_led(color, duration=0):
    if cleaned_up:
        return
    color_map = {
        'R':{'R':1,'G':0,'B':0},'G':{'R':0,'G':1,'B':0},
        'B':{'R':0,'G':0,'B':1},'W':{'R':1,'G':1,'B':1},
        'OFF':{'R':0,'G':0,'B':0},
    }
    states = color_map.get(color.upper(), color_map['OFF'])
    try:
        GPIO.output(LED_PINS['R'], states['R'])
        GPIO.output(LED_PINS['G'], states['G'])
        GPIO.output(LED_PINS['B'], states['B'])
    except RuntimeError:
        pass
    if duration > 0:
        time.sleep(duration)
        rgb_led('OFF')

def rgb_success():
    rgb_led('R', 0.3)
    rgb_led('G', 0.3)
    rgb_led('B', 0.3)
    rgb_led('W', 0.3)
    rgb_led('G', 0.5)

# ============ T265 ============
def activate_t265():
    try:
        import pyrealsense2 as rs
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            return False
        for dev in devices:
            print(f"✓ {dev.get_info(rs.camera_info.name)} (SN: {dev.get_info(rs.camera_info.serial_number)})")
        return True
    except ImportError:
        print("✗ pyrealsense2 未安装")
        return False
    except Exception as e:
        print(f"✗ {e}")
        return False

def cleanup_and_exit(i2c):
    global cleaned_up
    if cleaned_up:
        return
    cleaned_up = True
    
    rgb_led('OFF')
    time.sleep(0.1)
    
    try:
        GPIO.remove_event_detect(BUTTON_PIN)
    except:
        pass
    
    time.sleep(0.1)
    
    try:
        GPIO.cleanup()
    except:
        pass
    
    if i2c:
        try:
            oled_clear(i2c)
            i2c.close()
        except:
            pass
    
    sys.exit(0)

# ============ 主程序 ============
def main():
    try:
        i2c = I2C(OLED_ADDR, I2C_BUS)
        oled_init(i2c)
        oled_clear(i2c)
        print("OLED OK")
    except:
        i2c = None
    
    activated = False
    
    def on_press(channel):
        nonlocal activated
        if activated:
            return
        
        print("\n>>> 激活T265...")
        
        if i2c:
            oled_clear(i2c)
            draw_text(i2c, "ACTIVATING...", 2, 0)
        
        success = activate_t265()
        
        if success:
            activated = True
            print("✓ 成功！")
            if i2c:
                oled_clear(i2c)
                draw_text(i2c, "T265 READY!", 1, 0)
                draw_text(i2c, "============", 3, 0)
            rgb_success()
            cleanup_and_exit(i2c)
        else:
            print("✗ 失败")
            if i2c:
                oled_clear(i2c)
                draw_text(i2c, "FAILED!", 3, 0)
                draw_text(i2c, "PRESS BTN", 5, 0)
    
    GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=on_press, bouncetime=400)
    
    print("=" * 40)
    print("按键激活T265 - 按按键激活后自动退出")
    print("=" * 40)
    
    if i2c:
        oled_clear(i2c)
        draw_text(i2c, "PRESS BTN", 2, 0)
        draw_text(i2c, "TO ACTIVATE", 4, 0)
        draw_text(i2c, "T265", 5, 0)
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n退出")
        cleanup_and_exit(i2c)

if __name__ == "__main__":
    main()