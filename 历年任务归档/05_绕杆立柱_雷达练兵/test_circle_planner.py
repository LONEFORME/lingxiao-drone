"""circle_planner.generate_circle_waypoints() 单元测试。

运行:
    cd drone_control/circle_pole && python -m pytest test_circle_planner.py -v
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import pytest

from Lcode.circle_planner import generate_circle_waypoints, compute_detour_waypoint


class TestGenerateCircleWaypoints:
    def test_point_count_is_n_points_plus_closing_point(self):
        pts = generate_circle_waypoints(0.0, 0.0, 1.0, 0.0, radius=0.5, n_points=6)
        assert len(pts) == 7

    def test_last_point_closes_loop_back_to_first(self):
        pts = generate_circle_waypoints(0.0, 0.0, 1.0, 0.0, radius=0.5, n_points=6)
        assert pts[-1] == pts[0]

    def test_all_points_at_exact_radius_from_center(self):
        pts = generate_circle_waypoints(1.0, -0.5, 2.0, -0.5, radius=0.5, n_points=6)
        for x, y, _z in pts:
            d = math.hypot(x - 1.0, y - (-0.5))
            assert d == pytest.approx(0.5, abs=1e-9)

    def test_first_point_is_nearest_point_on_circle_to_current_position(self):
        # 飞机在圆心正右方3m处，最近的圆上点应该也在正右方(圆心+半径,0)
        pts = generate_circle_waypoints(0.0, 0.0, 3.0, 0.0, radius=0.5, n_points=6)
        assert pts[0][0] == pytest.approx(0.5, abs=1e-9)
        assert pts[0][1] == pytest.approx(0.0, abs=1e-9)

    def test_cw_direction_decreases_angle_top_view(self):
        pts = generate_circle_waypoints(0.0, 0.0, 0.5, 0.0, radius=0.5, n_points=4, direction="cw")
        # 从(0.5,0)开始，顺时针(顶视)下一个点应转到(0,-0.5)附近(角度0°->-90°)
        assert pts[1][0] == pytest.approx(0.0, abs=1e-6)
        assert pts[1][1] == pytest.approx(-0.5, abs=1e-6)

    def test_ccw_direction_increases_angle_top_view(self):
        pts = generate_circle_waypoints(0.0, 0.0, 0.5, 0.0, radius=0.5, n_points=4, direction="ccw")
        assert pts[1][0] == pytest.approx(0.0, abs=1e-6)
        assert pts[1][1] == pytest.approx(0.5, abs=1e-6)

    def test_z_passed_through_to_all_points(self):
        pts = generate_circle_waypoints(0.0, 0.0, 1.0, 0.0, radius=0.5, n_points=6, z=1.35)
        assert all(p[2] == 1.35 for p in pts)

    def test_current_position_at_center_falls_back_to_angle_zero(self):
        pts = generate_circle_waypoints(0.0, 0.0, 0.0, 0.0, radius=0.5, n_points=6)
        assert pts[0][0] == pytest.approx(0.5, abs=1e-9)
        assert pts[0][1] == pytest.approx(0.0, abs=1e-9)

    def test_rejects_too_few_points(self):
        with pytest.raises(ValueError):
            generate_circle_waypoints(0.0, 0.0, 1.0, 0.0, n_points=2)

    def test_rejects_invalid_direction(self):
        with pytest.raises(ValueError):
            generate_circle_waypoints(0.0, 0.0, 1.0, 0.0, direction="sideways")


class TestComputeDetourWaypoint:
    def test_returns_none_when_path_already_clear(self):
        # 杆塔在(5,5)，航线从(0,0)到(1,0)完全不靠近它
        d = compute_detour_waypoint(5.0, 5.0, 0.0, 0.0, 1.0, 0.0, safety_radius=0.75)
        assert d is None

    def test_returns_detour_when_path_crosses_safety_circle(self):
        # 杆塔正好在航线中点上，半径0.75m必然被穿过
        d = compute_detour_waypoint(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, safety_radius=0.75)
        assert d is not None

    def test_detour_point_is_outside_safety_radius_plus_margin(self):
        d = compute_detour_waypoint(1.0, 0.0, 0.0, 0.0, 2.0, 0.0,
                                     safety_radius=0.75, margin=0.15)
        dist = math.hypot(d[0] - 1.0, d[1] - 0.0)
        assert dist == pytest.approx(0.75 + 0.15, abs=1e-9)

    def test_detour_point_pushed_perpendicular_from_closest_approach(self):
        # 杆塔在直线正中间(1,0)上，航线沿x轴，最近点就是杆塔本身正上方——
        # 退化情况下随便选一个垂直方向推出，只要求确实在安全圈外即可
        d = compute_detour_waypoint(1.0, 0.0, 0.0, 0.0, 2.0, 0.0, safety_radius=0.75)
        assert math.hypot(d[0] - 1.0, d[1] - 0.0) >= 0.75

    def test_detour_point_on_same_side_as_near_miss(self):
        # 杆塔偏离直线0.3m(在安全半径0.75m以内)，最近点明显偏向杆塔那一侧，
        # 绕行点应该被推向"杆塔中心->最近点"方向的更远处，也就是继续偏移同一侧
        d = compute_detour_waypoint(1.0, 0.3, 0.0, 0.0, 2.0, 0.0, safety_radius=0.75)
        assert d[1] < 0.3  # 从杆塔中心朝(1,0)方向(即-y方向)推出去，y应该比0.3更小

    def test_zero_length_segment_returns_none(self):
        d = compute_detour_waypoint(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, safety_radius=0.75)
        assert d is None
