import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from Lcode.coverage_path import generate_boustrophedon_waypoints, CRUISE_Z_M


class TestGenerateBoustrophedonWaypoints:
    def test_returns_10_turn_waypoints(self):
        wps = generate_boustrophedon_waypoints()
        assert len(wps) == 10

    def test_first_waypoint_is_row0_start_at_origin(self):
        wps = generate_boustrophedon_waypoints()
        assert wps[0] == [0.0, 0.0, CRUISE_Z_M]

    def test_row0_end_at_x4_y0(self):
        wps = generate_boustrophedon_waypoints()
        assert wps[1] == [4.0, 0.0, CRUISE_Z_M]

    def test_lanes_alternate_direction_boustrophedon(self):
        wps = generate_boustrophedon_waypoints()
        # row1: 4.0,0.8 -> 0.0,0.8 (反向)
        assert wps[2] == [4.0, 0.8, CRUISE_Z_M]
        assert wps[3] == [0.0, 0.8, CRUISE_Z_M]

    def test_last_waypoint_is_row4_end_at_x4_y3_2(self):
        wps = generate_boustrophedon_waypoints()
        assert wps[9] == [4.0, 3.2, CRUISE_Z_M]

    def test_row_y_spacing_is_0_8m(self):
        wps = generate_boustrophedon_waypoints()
        ys = [wps[i][1] for i in (0, 2, 4, 6, 8)]
        assert ys == [0.0, 0.8, 1.6, 2.4, 3.2]

    def test_num_rows_and_cols_are_configurable(self):
        wps = generate_boustrophedon_waypoints(num_cols=3, num_rows=2, cell_size_m=1.0)
        # 2行3列 -> col跨度(3-1)*1.0=2.0m, row跨度(2-1)*1.0=1.0m, 2行*2转弯=4个航点
        assert len(wps) == 4
        assert wps[0] == [0.0, 0.0, CRUISE_Z_M]
        assert wps[1] == [2.0, 0.0, CRUISE_Z_M]
        assert wps[3] == [0.0, 1.0, CRUISE_Z_M]
