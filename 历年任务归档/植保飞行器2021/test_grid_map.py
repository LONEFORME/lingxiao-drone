"""测试网格坐标映射。"""
import json
import pytest
from pathlib import Path

from Lcode.grid_map import GridMap


def _write_config(tmp_path: Path) -> Path:
    config = {
        "cruise_height_m": 1.5,
        "home": {"x": 0.0, "y": 0.0},
        "rows": [
            {"row_y": 0.25, "cols": [1, 2]},
            {"row_y": 0.75, "cols": [3, 4, 5, 6]},
        ],
        "start_grid": 2,
    }
    path = tmp_path / "config.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    return path


@pytest.mark.fast
def test_loads_all_cells(tmp_path):
    gm = GridMap(_write_config(tmp_path))
    assert len(gm) == 6  # 1-6


@pytest.mark.fast
def test_cell_center_coordinates(tmp_path):
    gm = GridMap(_write_config(tmp_path))
    center = gm.get_center(1)
    assert center is not None
    x, y, z = center
    assert x == -0.75
    assert y == 0.25
    assert z == 1.5


@pytest.mark.fast
def test_start_grid(tmp_path):
    gm = GridMap(_write_config(tmp_path))
    assert gm.start_grid == 2


@pytest.mark.fast
def test_spray_grids(tmp_path):
    gm = GridMap(_write_config(tmp_path))
    assert gm.spray_grids() == [1, 2, 3, 4, 5, 6]


@pytest.mark.fast
def test_home_position(tmp_path):
    gm = GridMap(_write_config(tmp_path))
    hx, hy, hz = gm.home
    assert hx == 0.0
    assert hy == 0.0
    assert hz == 1.5


@pytest.mark.fast
def test_rows_property(tmp_path):
    gm = GridMap(_write_config(tmp_path))
    assert gm.rows == [[1, 2], [3, 4, 5, 6]]


@pytest.mark.fast
def test_missing_cell_returns_none(tmp_path):
    gm = GridMap(_write_config(tmp_path))
    assert gm.get_cell(99) is None
    assert gm.get_center(99) is None


@pytest.mark.fast
def test_non_spray_overrides(tmp_path):
    config = {
        "cruise_height_m": 1.5,
        "home": {"x": 0.0, "y": 0.0},
        "rows": [{"row_y": 0.25, "cols": [1, 2]}],
        "non_spray": [{"id": 2}],
    }
    path = tmp_path / "config.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    gm = GridMap(path)
    assert gm.get_cell(2) is not None
    assert not gm.get_cell(2).spray
    assert gm.spray_grids() == [1]
