"""全覆盖路径规划 — 植保飞行器的 S 型播撒路径生成。

策略：
  1. 从起点网格 (21/A) 出发
  2. 沿 S 型访问所有播撒区网格
  3. 最后回到起降点
"""
from __future__ import annotations

from typing import Optional

from Lcode.grid_map import GridMap


def plan_coverage_path(
    grid_map: GridMap,
    start_grid: Optional[int] = None,
) -> list[tuple[float, float, float]]:
    """生成全覆盖路径航点列表。

    每个航点格式: (x, y, z) 单位米。
    第一个航点是起降点起飞高度，最后一个是起降点高度。
    """
    if start_grid is None:
        start_grid = grid_map.start_grid
    if start_grid is None:
        return []

    rows = _rows_containing_spray(grid_map)
    if not rows:
        return []

    ordered = _snake_from_start(rows, start_grid)

    waypoints: list[tuple[float, float, float]] = []

    # 1. 起降点 → 巡航高度
    waypoints.append(grid_map.home)

    # 2. 各网格中心（按 S 型顺序）
    for gid in ordered:
        center = grid_map.get_center(gid)
        if center is not None:
            waypoints.append(center)

    # 3. 返回起降点
    waypoints.append(grid_map.home)

    return waypoints


def _rows_containing_spray(grid_map: GridMap) -> list[list[int]]:
    """返回仅含播撒网格的行分组。"""
    spray_set = set(grid_map.spray_grids())
    result: list[list[int]] = []
    for row in grid_map.rows:
        filtered = [gid for gid in row if gid in spray_set]
        if filtered:
            result.append(filtered)
    return result


def _snake_from_start(rows: list[list[int]], start: int) -> list[int]:
    """按 S 型排列网格，从 start 出发，覆盖所有行。

    S 型规则：
      - 从 start 所在行出发
      - 奇数行（从 start 行计为第0行）：从左到右
      - 偶数行：从右到左
      - 先往远（Y 递增）走到尽头，再从远往回走到近处
    """
    # 找 start 所在行索引
    start_row_idx = None
    for i, row in enumerate(rows):
        if start in row:
            start_row_idx = i
            break
    if start_row_idx is None:
        # 兜底：直接展平所有行
        return [gid for row in rows for gid in row]

    ordered: list[int] = []

    # 阶段 1：从 start 行向远走（Y 递增）
    for i in range(start_row_idx, len(rows)):
        row = rows[i]
        # 从 start 行开始，方向交替
        if (i - start_row_idx) % 2 == 0:
            # 偶数步：从左到右
            ordered.extend(sorted(row))
        else:
            # 奇数步：从右到左
            ordered.extend(sorted(row, reverse=True))

    # 阶段 2：从 start 行往回走（Y 递减）
    for i in range(start_row_idx - 1, -1, -1):
        row = rows[i]
        if (start_row_idx - i) % 2 == 0:
            ordered.extend(sorted(row, reverse=True))
        else:
            ordered.extend(sorted(row))

    # 去重（保留首次出现顺序）
    seen: set[int] = set()
    result: list[int] = []
    for gid in ordered:
        if gid not in seen:
            seen.add(gid)
            result.append(gid)

    return result
