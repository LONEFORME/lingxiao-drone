"""植保飞行器网格坐标系 — 1~28 号网格到场地坐标的映射。

场地 500×200cm，网格 50×50cm，起降点十字中心为坐标原点 (0,0)。

坐标约定：
  X 轴：水平向右为正
  Y 轴：场地纵深（远离起降点）为正
  Z 轴：垂直向上为正
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Optional


@dataclass(frozen=True)
class GridCell:
    id: int
    x: float      # 网格中心 X (m)
    y: float      # 网格中心 Y (m)
    spray: bool   # 是否为播撒区（绿色）


class GridMap:
    """管理 1-28 号网格的坐标与属性，以及行分组。"""

    def __init__(self, config_path: Optional[Path] = None):
        self._cells: dict[int, GridCell] = {}
        self._rows: list[list[int]] = []  # 每行的 grid_id 列表
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_hold_s = 1.0
        self.cruise_height_m = 1.5
        self.start_grid_id: Optional[int] = 21
        self.laser_spray_times = 2
        self.laser_flash_s = 0.5
        self.laser_interval_s = 0.8

        if config_path and config_path.exists():
            self._load_config(config_path)

    # ── 加载配置 ──────────────────────────────────────────

    def _load_config(self, path: Path) -> None:
        raw = json.loads(path.read_text(encoding="utf-8"))
        self.cruise_height_m = float(raw.get("cruise_height_m", 1.5))
        self.home_hold_s = float(raw.get("home_hold_s", 1.0))
        self.laser_spray_times = int(raw.get("laser_spray_times", 2))
        self.laser_flash_s = float(raw.get("laser_flash_s", 0.5))
        self.laser_interval_s = float(raw.get("laser_interval_s", 0.8))
        self.start_grid_id = raw.get("start_grid", 21)

        home = raw.get("home", {})
        self.home_x = float(home.get("x", 0.0))
        self.home_y = float(home.get("y", 0.0))

        # 从 rows 构建网格，同时保留行分组
        x_start = float(raw.get("default_cell_x_start", -0.75))
        x_step = float(raw.get("cell_spacing_x", 0.5))

        self._rows = []
        for row in raw.get("rows", []):
            row_y = float(row["row_y"])
            row_ids: list[int] = []
            for i, gid in enumerate(row["cols"]):
                grid_id = int(gid)
                cell = GridCell(
                    id=grid_id,
                    x=x_start + i * x_step,
                    y=row_y,
                    spray=True,
                )
                self._cells[cell.id] = cell
                row_ids.append(grid_id)
            if row_ids:
                self._rows.append(row_ids)

        # 读取非播撒区覆盖（发挥① 用）
        for g in raw.get("non_spray", []):
            gid = int(g["id"])
            if gid in self._cells:
                self._cells[gid] = GridCell(
                    id=gid,
                    x=self._cells[gid].x,
                    y=self._cells[gid].y,
                    spray=False,
                )

    # ── 查询 ──────────────────────────────────────────────

    def get_cell(self, grid_id: int) -> Optional[GridCell]:
        return self._cells.get(grid_id)

    def get_center(self, grid_id: int) -> Optional[tuple[float, float, float]]:
        cell = self.get_cell(grid_id)
        if cell is None:
            return None
        return (cell.x, cell.y, self.cruise_height_m)

    def spray_grids(self) -> list[int]:
        return sorted(
            gid for gid, c in self._cells.items() if c.spray
        )

    @property
    def rows(self) -> list[list[int]]:
        """返回每行的 grid_id 列表，从近到远（Y 递增）。"""
        return list(self._rows)

    @property
    def home(self) -> tuple[float, float, float]:
        return (self.home_x, self.home_y, self.cruise_height_m)

    @property
    def start_grid(self) -> Optional[int]:
        if self.start_grid_id and self.start_grid_id in self._cells:
            return self.start_grid_id
        spray = self.spray_grids()
        return spray[0] if spray else None

    def __len__(self) -> int:
        return len(self._cells)

    def __contains__(self, grid_id: int) -> bool:
        return grid_id in self._cells

    def __iter__(self):
        return iter(sorted(self._cells.values(), key=lambda c: c.id))
