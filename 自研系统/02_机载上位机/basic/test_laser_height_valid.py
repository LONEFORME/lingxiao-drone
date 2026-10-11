"""激光高度合理性上限过滤(laser_height_valid)的单元测试。

背景：2026-07-10真机降落回归测试(basic_radar)发现，降落末尾激光传感器偶发
返回类似0xFFFFFFFF的错误码，`_last_laser_height_cm`除以100后变成约4.29e7
米的垃圾值。原逻辑(loop()/land()里)只判断 laser_h > 0.05，没有上限，垃圾值
被直接当真实高度写进 pos[2]/land_pos[2]，污染飞行日志的高度曲线。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/basic && python -m pytest test_laser_height_valid.py -v
"""
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from Mission_GPT import laser_height_valid, LASER_HEIGHT_MAX_M


class TestLaserHeightValid:
    def test_rejects_overflow_garbage_value(self):
        """2026-07-10真机实测捕获到的真实垃圾值：0xFFFFFFFF/100 ≈ 4.29e7米。"""
        assert laser_height_valid(42949672.95) is False

    def test_accepts_normal_indoor_height(self):
        assert laser_height_valid(1.03) is True

    def test_rejects_at_or_below_noise_floor(self):
        assert laser_height_valid(0.05) is False
        assert laser_height_valid(0.0) is False

    def test_accepts_value_at_upper_bound(self):
        assert laser_height_valid(LASER_HEIGHT_MAX_M) is True

    def test_rejects_value_above_upper_bound(self):
        assert laser_height_valid(LASER_HEIGHT_MAX_M + 0.01) is False
