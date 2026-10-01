"""测试全覆盖路径规划。"""
import json
import pytest
from pathlib import Path

from Lcode.grid_map import GridMap
from Lcode.coverage_planner import plan_coverage_path


def _make_grid(tmp_path, rows_data, start=21):
    """创建带 rows 的 GridMap。"""
    config = {
        "cruise_height_m": 1.5,
        "home": {"x": 0.0, "y": 0.0},
        "rows": rows_data,
        "start_grid": start,
    }
    path = tmp_path / "config.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    return GridMap(path)


@pytest.mark.fast
def test_plan_returns_waypoints_with_home_start_end(tmp_path):
    """路径以起降点开始和结束。"""
    rows = [
        {"row_y": 3.25, "cols": [21, 22]},
        {"row_y": 3.75, "cols": [23, 24]},
    ]
    gm = _make_grid(tmp_path, rows, start=21)
    path = plan_coverage_path(gm)
    assert len(path) >= 3
    # 第一个和最后一个都是起降点
    assert path[0] == gm.home
    assert path[-1] == gm.home


@pytest.mark.fast
def test_snake_order_starts_at_start_grid(tmp_path):
    """S 型路径从 start_grid 出发。"""
    rows = [
        {"row_y": 3.25, "cols": [21, 22, 23, 24]},
        {"row_y": 3.75, "cols": [25, 26, 27, 28]},
    ]
    gm = _make_grid(tmp_path, rows, start=21)
    path = plan_coverage_path(gm)

    # 第一个航点后应该到 21 号格中心
    assert len(path) >= 2
    c21 = gm.get_center(21)
    assert path[1] == c21


@pytest.mark.fast
def test_covers_all_spray_grids_once(tmp_path):
    """路径覆盖所有播撒网格各一次。"""
    rows = [
        {"row_y": 0.25, "cols": [1, 2]},
        {"row_y": 0.75, "cols": [3, 4, 5, 6]},
        {"row_y": 1.25, "cols": [7, 8, 9, 10]},
    ]
    gm = _make_grid(tmp_path, rows, start=7)
    path = plan_coverage_path(gm)

    # 去掉起降点航点，提取网格中心
    grid_centers = set()
    for wp in path[1:-1]:
        for gid in gm.spray_grids():
            if wp == gm.get_center(gid):
                grid_centers.add(gid)
                break

    assert grid_centers == set(gm.spray_grids())


@pytest.mark.fast
def test_empty_grid_map_returns_empty(tmp_path):
    """没有网格时返回空列表。"""
    config = {
        "cruise_height_m": 1.5,
        "home": {"x": 0.0, "y": 0.0},
        "rows": [],
    }
    path = tmp_path / "config.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    gm = GridMap(path)
    path_plan = plan_coverage_path(gm)
    assert path_plan == []


@pytest.mark.fast
def test_snake_direction_alternates_per_row(tmp_path):
    """相邻行方向交替。"""
    rows = [
        {"row_y": 3.25, "cols": [21, 22]},
        {"row_y": 3.75, "cols": [23, 24]},
    ]
    gm = _make_grid(tmp_path, rows, start=21)
    path = plan_coverage_path(gm)

    centers = path[1:-1]  # 去掉起降点
    # 第一行 (21→22): 从左到右
    # 第二行 (24→23): 从右到左
    # 验证：21 在 22 之前，23 在 24 之后
    c21 = gm.get_center(21)
    c22 = gm.get_center(22)
    c23 = gm.get_center(23)
    c24 = gm.get_center(24)

    assert centers.index(c21) < centers.index(c22)
    assert centers.index(c24) < centers.index(c23)
