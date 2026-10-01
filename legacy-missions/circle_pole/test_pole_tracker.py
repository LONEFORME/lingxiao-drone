"""PoleTracker 世界系重构单元测试。

运行（先确保已 pip install pytest pyserial）：
    cd drone_control/circle_pole && python -m pytest test_pole_tracker.py -v
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Lcode.Lradar import (
    radar_angle_to_body_xy,
    body_to_world_xy,
    world_to_body_angle_dist,
)


# ════════════════════ body_to_world_xy / world_to_body_angle_dist ════════════════════

class TestBodyWorldTransform:
    def test_zero_yaw_identity_shift(self):
        # yaw=0 时，世界系就是机体系平移了飞机当前位置
        bx, by = radar_angle_to_body_xy(0, 1000)  # 机体正前方1m
        wx, wy = body_to_world_xy(2.0, -1.0, 0.0, bx, by, yaw_sign=1)
        assert wx == pytest.approx(3.0, abs=1e-6)
        assert wy == pytest.approx(-1.0, abs=1e-6)

    def test_yaw_90deg_sign_positive(self):
        bx, by = radar_angle_to_body_xy(0, 1000)  # bx=1.0, by=0.0
        wx, wy = body_to_world_xy(0.0, 0.0, math.pi / 2, bx, by, yaw_sign=1)
        assert wx == pytest.approx(0.0, abs=1e-6)
        assert wy == pytest.approx(1.0, abs=1e-6)

    def test_yaw_90deg_sign_negative(self):
        # yaw_sign 翻转应该翻转旋转方向，这是给以后真机标定用的开关
        bx, by = radar_angle_to_body_xy(0, 1000)
        wx, wy = body_to_world_xy(0.0, 0.0, math.pi / 2, bx, by, yaw_sign=-1)
        assert wx == pytest.approx(0.0, abs=1e-6)
        assert wy == pytest.approx(-1.0, abs=1e-6)

    def test_round_trip_recovers_original_angle_distance(self):
        # world_to_body_angle_dist 是 body_to_world_xy 的逆变换：
        # 任意机体系候选点转到世界系、再转回来，应该拿回原始角度/距离
        angle_deg, distance_mm = 30.0, 800.0
        x_m, y_m, yaw_rad, sign = 2.0, -1.5, math.radians(40), 1
        bx, by = radar_angle_to_body_xy(angle_deg, distance_mm)
        wx, wy = body_to_world_xy(x_m, y_m, yaw_rad, bx, by, yaw_sign=sign)
        angle2, dist2 = world_to_body_angle_dist(wx, wy, x_m, y_m, yaw_rad, yaw_sign=sign)
        assert angle2 == pytest.approx(angle_deg, abs=1e-4)
        assert dist2 == pytest.approx(distance_mm, abs=1e-4)

    def test_round_trip_with_negative_sign(self):
        angle_deg, distance_mm = 200.0, 650.0
        x_m, y_m, yaw_rad, sign = -0.6, 0.07, math.radians(3.0), -1
        bx, by = radar_angle_to_body_xy(angle_deg, distance_mm)
        wx, wy = body_to_world_xy(x_m, y_m, yaw_rad, bx, by, yaw_sign=sign)
        angle2, dist2 = world_to_body_angle_dist(wx, wy, x_m, y_m, yaw_rad, yaw_sign=sign)
        assert angle2 == pytest.approx(angle_deg, abs=1e-4)
        assert dist2 == pytest.approx(distance_mm, abs=1e-4)


# ════════════════════ PoleTracker 世界系匹配 ════════════════════

class FakeRadar:
    """模拟 Serial_radar.get_scan()，只提供 PoleTracker.update() 需要的接口。"""
    def __init__(self):
        self._scan = {}

    def set_single_point(self, angle_deg, distance_mm, intensity=100):
        self._scan = {round(angle_deg) % 360: (distance_mm, intensity)}

    def set_empty(self):
        self._scan = {}

    def get_scan(self):
        return dict(self._scan)


class TestPoleTrackerWorldFrame:
    def test_confirms_static_pole_despite_yaw_rotation(self):
        from Lcode.Lradar import PoleTracker

        pole_world = (1.0, 0.0)
        tracker = PoleTracker(window=6, min_hits=3, world_eps_m=0.2)
        radar = FakeRadar()

        # 飞机原地不动，但朝向在几次轮询之间明显变化
        for yaw_deg in (0.0, 15.0, -10.0, 20.0):
            yaw_rad = math.radians(yaw_deg)
            angle, dist_mm = world_to_body_angle_dist(
                pole_world[0], pole_world[1], 0.0, 0.0, yaw_rad, yaw_sign=1
            )
            radar.set_single_point(angle, dist_mm)
            tracker.update(radar, 0.0, 0.0, yaw_rad)

        confirmed = tracker.confirmed_poles()
        assert len(confirmed) == 1
        assert confirmed[0]["x"] == pytest.approx(pole_world[0], abs=0.05)
        assert confirmed[0]["y"] == pytest.approx(pole_world[1], abs=0.05)
        assert confirmed[0]["hits"] >= 3

    def test_confirms_static_pole_despite_off_axis_approach(self):
        """复现2026-07-07真机失败场景的简化版：飞机从(0,0)飞向(-0.6,0.065)，
        杆子在(-1.0,0.0)，不严格在正前方，方位角会随之摆动。"""
        from Lcode.Lradar import PoleTracker

        pole_world = (-1.0, 0.0)
        waypoints = [(0.0, 0.0), (-0.2, 0.02), (-0.4, 0.045), (-0.6, 0.065)]
        yaw_rad = math.radians(2.0)  # 全程yaw基本不变，跟真实数据一致

        angles = []
        for x, y in waypoints:
            angle, _ = world_to_body_angle_dist(pole_world[0], pole_world[1], x, y, yaw_rad, yaw_sign=1)
            angles.append(angle)
        # 先确认这组坐标真的复现了"方位角明显摆动"这个前提，摆动应该超过旧版4°容差
        # (实测这组坐标产生约9.2°摆动，仍远超旧版4°容差，虽不到当初预想的10°)
        assert max(angles) - min(angles) > 8.0

        tracker = PoleTracker(window=6, min_hits=3, world_eps_m=0.2)
        radar = FakeRadar()
        for x, y in waypoints:
            angle, dist_mm = world_to_body_angle_dist(pole_world[0], pole_world[1], x, y, yaw_rad, yaw_sign=1)
            radar.set_single_point(angle, dist_mm)
            tracker.update(radar, x, y, yaw_rad)

        confirmed = tracker.confirmed_poles()
        assert len(confirmed) == 1
        assert confirmed[0]["x"] == pytest.approx(pole_world[0], abs=0.05)
        assert confirmed[0]["y"] == pytest.approx(pole_world[1], abs=0.05)

    def test_non_repeating_noise_not_confirmed(self):
        from Lcode.Lradar import PoleTracker

        tracker = PoleTracker(window=6, min_hits=3, world_eps_m=0.2)
        radar = FakeRadar()
        # 4次轮询，每次一个互相离得很远(>world_eps_m)的候选点，模拟不重复出现的噪声
        noise_angles_dists = [(10, 900), (120, 700), (250, 850), (300, 600)]
        for angle, dist_mm in noise_angles_dists:
            radar.set_single_point(angle, dist_mm)
            tracker.update(radar, 0.0, 0.0, 0.0)

        assert tracker.confirmed_poles() == []
