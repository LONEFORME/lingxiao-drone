"""T265原始加速度计/陀螺仪数据接口的单元测试。

背景：2026-07-10发现降落确认(unlock_sta/motor_pwm_mask)可能存在凌霄IMU固件
层面的问题——真机测试出现过两个字段都读到"已停转"新鲜数据、但用户确认电机
实际仍在转的案例。计划用T265自带的加速度计/陀螺仪振动特征做独立验证(完全
不依赖凌霄IMU自己上报的任何遥测字段)：电机转动会有特征性振动，理论上能从
这份数据独立判断"电机是否还在转"。此前t265.py只暴露融合后的position/
velocity/orientation，没有原始IMU接口，这里补上。

运行（先确保已 pip install pytest）：
    cd drone_control/circle_pole && python -m pytest test_t265_raw_imu.py -v
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(__file__))

from t265 import t265_class


class TestRawImu:
    def test_get_raw_imu_returns_six_floats_in_simulation_mode(self):
        """模拟模式(无pyrealsense2或强制走模拟分支)下，get_raw_imu()应该返回
        (ax, ay, az, gx, gy, gz)六个浮点数，不依赖真实硬件也能验证接口形状。"""
        t = t265_class()
        t.use_simulation = True
        t.start()
        time.sleep(0.1)
        imu = t.get_raw_imu()
        t.stop()

        assert len(imu) == 6
        assert all(isinstance(v, float) for v in imu)

    def test_get_raw_imu_returns_copy_not_internal_reference(self):
        """返回值应该是拷贝，调用方修改返回的数组不能影响内部状态(跟
        get_position()/get_velocity()已有的拷贝行为保持一致)。"""
        t = t265_class()
        t.use_simulation = True
        t.start()
        time.sleep(0.1)
        imu1 = t.get_raw_imu()
        imu1[0] = 999.0
        imu2 = t.get_raw_imu()
        t.stop()

        assert imu2[0] != 999.0

    def test_get_raw_imu_before_start_returns_zeros(self):
        """还没start()时不应该抛异常，返回初始零值。"""
        t = t265_class()
        imu = t.get_raw_imu()
        assert len(imu) == 6
        assert all(v == 0.0 for v in imu)
