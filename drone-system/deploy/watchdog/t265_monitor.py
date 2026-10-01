#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
T265 OLED Status Monitor
开机自启动，OLED显示T265状态、摄像头状态和本机IP
"""

import time
import subprocess
import os
import sys
from i2cdev import I2C

sys.path.insert(0, '/home/sunrise/Desktop/FJJ')
try:
    from competition_2026_d.vision.link_status import (
        OledLinkEvaluator,
        read_link_status,
    )
except Exception:
    OledLinkEvaluator = None
    read_link_status = None

I2C_BUS = 5
OLED_ADDR = 0x3C

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
    'I':[0x00,0x41,0x7F,0x41,0x00],'J':[0x20,0x40,0x41,0x3F,0x01],
    'K':[0x7F,0x08,0x14,0x22,0x41],'L':[0x7F,0x40,0x40,0x40,0x40],
    'M':[0x7F,0x02,0x0C,0x02,0x7F],'N':[0x7F,0x04,0x08,0x10,0x7F],
    'O':[0x3E,0x41,0x41,0x41,0x3E],'P':[0x7F,0x09,0x09,0x09,0x06],
    'Q':[0x3E,0x41,0x51,0x21,0x5E],'R':[0x7F,0x09,0x19,0x29,0x46],
    'S':[0x46,0x49,0x49,0x49,0x31],'T':[0x01,0x01,0x7F,0x01,0x01],
    'U':[0x3F,0x40,0x40,0x40,0x3F],'V':[0x1F,0x20,0x40,0x20,0x1F],
    'W':[0x3F,0x40,0x38,0x40,0x3F],'X':[0x63,0x14,0x08,0x14,0x63],
    'Y':[0x07,0x08,0x70,0x08,0x07],'Z':[0x61,0x51,0x49,0x45,0x43],
    '0':[0x3E,0x51,0x49,0x45,0x3E],'1':[0x00,0x42,0x7F,0x40,0x00],
    '2':[0x42,0x61,0x51,0x49,0x46],'3':[0x21,0x41,0x45,0x4B,0x31],
    '4':[0x18,0x14,0x12,0x7F,0x10],'5':[0x27,0x45,0x45,0x45,0x39],
    '6':[0x3C,0x4A,0x49,0x49,0x30],'7':[0x01,0x71,0x09,0x05,0x03],
    '8':[0x36,0x49,0x49,0x49,0x36],'9':[0x06,0x49,0x49,0x29,0x1E],
    ' ':[0x00,0x00,0x00,0x00,0x00],'.':[0x00,0x60,0x60,0x00,0x00],
    '!':[0x00,0x5F,0x00,0x00,0x00],'-':[0x08,0x08,0x08,0x08,0x08],
    '=':[0x14,0x14,0x14,0x14,0x14],':':[0x00,0x36,0x36,0x00,0x00],
    '/':[0x20,0x10,0x08,0x04,0x02],'+':[0x08,0x08,0x3E,0x08,0x08],
    '%':[0x23,0x13,0x08,0x64,0x62],'(':[0x00,0x1C,0x22,0x41,0x00],
    ')':[0x00,0x41,0x22,0x1C,0x00],
    '>':[0x00,0x41,0x22,0x14,0x08],
}

def draw_text(i2c, text, page, col=0):
    for char in text:
        if col >= 128:
            break
        c = char.upper()
        if c in FONT:
            oled_set_pos(i2c, page, col)
            for i in range(5):
                oled_write_data(i2c, FONT[c][i])
            oled_write_data(i2c, 0x00)
        col += 6

# ============ Status ============
def get_ip():
    try:
        result = subprocess.run(
            ['hostname', '-I'],
            capture_output=True,
            text=True,
            timeout=3
        )
        parts = result.stdout.strip().split()
        if parts:
            return parts[0]
        return 'N/A'
    except:
        return 'N/A'

def get_t265_status():
    try:
        result = subprocess.run(
            ['lsusb'],
            capture_output=True,
            text=True,
            timeout=5
        )

        if '8087:0b37' in result.stdout:
            return 'READY'
        elif '03e7:2150' in result.stdout:
            return 'NEED REPLUG'
        else:
            return 'NOT FOUND'
    except:
        return 'ERROR'

def get_video0_status():
    try:
        if os.path.exists('/dev/video0'):
            return 'CONNECTED'
        return 'NOT FOUND'
    except:
        return 'ERROR'

# ============ Main ============
def main():
    i2c = None

    for retry in range(5):
        try:
            i2c = I2C(OLED_ADDR, I2C_BUS)
            oled_init(i2c)
            oled_clear(i2c)
            break
        except:
            time.sleep(2)

    if i2c is None:
        print("OLED init failed")
        return

    # 启动阶段显示检查中
    draw_text(i2c, "T265:CHECKING", 0, 0)
    draw_text(i2c, "VIDEO0:CHECKING", 1, 0)
    draw_text(i2c, "IP:CHECKING", 2, 0)
    draw_text(i2c, "CAM>RDK:LOST", 3, 0)
    draw_text(i2c, "RDK>CAM:LOST", 4, 0)

    last_display = {
        0: 'T265:CHECKING',
        1: 'VIDEO0:CHECKING',
        2: 'IP:CHECKING',
        3: 'CAM>RDK:LOST',
        4: 'RDK>CAM:LOST',
    }
    link_evaluator = OledLinkEvaluator() if OledLinkEvaluator is not None else None

    while True:
        # 获取设备状态
        t265_status = get_t265_status()
        video0_status = get_video0_status()
        ip = get_ip()
        if link_evaluator is None or read_link_status is None:
            link_states = {'cam_to_rdk': 'LOST', 'rdk_to_cam': 'LOST'}
        else:
            link_states = link_evaluator.evaluate(read_link_status())

        # OLED 只显示这三项信息
        lines = {
            0: f'T265:{t265_status}',
            1: f'VIDEO0:{video0_status}',
            2: f'IP:{ip}',
            3: f"CAM>RDK:{link_states['cam_to_rdk']}",
            4: f"RDK>CAM:{link_states['rdk_to_cam']}",
            5: '',
            6: '',
            7: '',
        }

        # 只更新发生变化的行，避免全屏刷新导致闪烁
        if lines != last_display:
            for page, text in lines.items():
                old_text = last_display.get(page, '')

                if text != old_text:
                    # 清除当前行
                    oled_set_pos(i2c, page, 0)
                    for col in range(128):
                        oled_write_data(i2c, 0x00)

                    # 重新绘制当前行
                    if text:
                        draw_text(i2c, text, page, 0)

            last_display = lines.copy()

        time.sleep(1)


if __name__ == "__main__":
    main()
