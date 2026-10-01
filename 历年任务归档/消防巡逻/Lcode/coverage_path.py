"""6列x5行格心弓字形(boustrophedon)全覆盖巡逻航点生成。
见 docs/superpowers/specs/2026-07-16-fire-patrol-design.md "覆盖巡逻路径"一节。

赛题40dm x 48dm巡防区域按8dm x 8dm划分成6列x5行=30个格心，本地坐标系原点
(0,0,0)取T265上电点，起降点物理上放在row0/col0格心，格心间距0.8m。
只在每行两端的格心转弯，行内连续飞行(不逐格停留)。
"""
from typing import List

NUM_COLS = 6
NUM_ROWS = 5
CELL_SIZE_M = 0.8
CRUISE_Z_M = 1.8  # 对应赛题18dm巡航高度要求


def generate_boustrophedon_waypoints(num_cols: int = NUM_COLS,
                                      num_rows: int = NUM_ROWS,
                                      cell_size_m: float = CELL_SIZE_M,
                                      cruise_z_m: float = CRUISE_Z_M) -> List[List[float]]:
    """生成弓字形转弯点列表，每行2个航点(起点+终点)，共 num_rows*2 个。"""
    x_max = round((num_cols - 1) * cell_size_m, 6)
    waypoints = []
    for row in range(num_rows):
        y = round(row * cell_size_m, 6)
        if row % 2 == 0:
            x_start, x_end = 0.0, x_max
        else:
            x_start, x_end = x_max, 0.0
        waypoints.append([x_start, y, cruise_z_m])
        waypoints.append([x_end, y, cruise_z_m])
    return waypoints


def save_router_txt(path: str, waypoints: List[List[float]]) -> None:
    """写入 basic/Mission_GPT.py `load_waypoints()` 兼容的 x,y,z 逐行格式，
    末尾追加返回原点(0,0,0)降落航点。"""
    with open(path, "w", encoding="utf-8") as f:
        f.write("# x,y,z (meters) — fire_patrol 6x5格心弓字形全覆盖巡逻航点\n")
        f.write("# 自动生成，见 Lcode/coverage_path.py generate_boustrophedon_waypoints()\n")
        for x, y, z in waypoints:
            f.write(f"{x:.2f},{y:.2f},{z:.2f}\n")
        f.write("0.00,0.00,0.00\n")  # 返航降落点 = 本地原点


if __name__ == "__main__":
    import os
    wps = generate_boustrophedon_waypoints()
    out_path = os.path.join(os.path.dirname(__file__), "..", "router.txt")
    save_router_txt(out_path, wps)
    print(f"写入 {len(wps)}+1 个航点到 {out_path}")
