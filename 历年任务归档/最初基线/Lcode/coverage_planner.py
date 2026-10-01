"""
9x7 网格全覆盖路径规划器 - 类封装版
=========================
问题：
  9(A列) x 7(B行) 网格，从 (A9,B1) 出发覆盖全部可通行格子后回到起点
  有 3 个水平或竖直连续的禁飞格子
  输出飞行路径文件和 router 指令文件

坐标映射：
  内部(x,y)  <->  A??,B??  <->  实际值(米)
  (x=0, y=0)  ->  (A9, B1)  ->  (0, 0)
  (x=8, y=0)  ->  (A1, B1)  ->  (-4.0, 0)
  实际_x = -(x) * 0.5
  实际_y = (y) * 0.5
"""

import sys
import io
from collections import deque

if sys.stdout.encoding is None or sys.stdout.encoding.upper() != 'UTF-8':
    try:
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
    except Exception:
        pass


class CoveragePlanner:
    """9x7 网格全覆盖路径规划器"""

    WIDTH = 9
    HEIGHT = 7

    # ---- 坐标转换 (静态) ----
    @staticmethod
    def ab_to_xy(a_str, b_str):
        """'A??', 'B??' -> (x, y) 内部坐标"""
        return (9 - int(a_str[1:]), int(b_str[1:]) - 1)

    @staticmethod
    def xy_to_ab_str(xy):
        """(x, y) -> '(A??, B??)' 字符串"""
        x, y = xy
        return f"(A{9 - x}, B{y + 1})"

    @staticmethod
    def xy_to_real(xy):
        """(x, y) 内部坐标 -> (实际_x, 实际_y) 米"""
        x, y = xy
        return (-x * 0.5, y * 0.5)

    @staticmethod
    def real_to_xy(rx, ry):
        """实际坐标(米) -> (ix, iy) 内部格子坐标，超出范围返回 None"""
        ix = int(round(-rx * 2))
        iy = int(round( ry * 2))
        if 0 <= ix < CoveragePlanner.WIDTH and 0 <= iy < CoveragePlanner.HEIGHT:
            return (ix, iy)
        return None

    # ---- 初始化 ----
    def __init__(self, forbidden_zones_ab):
        """
        forbidden_zones_ab: [("A6","B3"), ("A5","B3"), ...]
            3个水平或竖直连续的格子，用 A??,B?? 格式
        """
        self.forbidden_zones_ab = list(forbidden_zones_ab)
        self.grid = self._build_grid()
        self.start_xy = (0, 0)  # (A9, B1)
        self.best_result = None

    def _build_grid(self):
        """构建网格，0=可通行，1=禁飞"""
        g = [[0] * self.HEIGHT for _ in range(self.WIDTH)]
        for a, b in self.forbidden_zones_ab:
            x, y = self.ab_to_xy(a, b)
            if 0 <= x < self.WIDTH and 0 <= y < self.HEIGHT:
                g[x][y] = 1
        return g

    # ---- 工具方法 ----
    def _get_neighbors(self, x, y):
        r = []
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.WIDTH and 0 <= ny < self.HEIGHT and self.grid[nx][ny] == 0:
                r.append((nx, ny))
        return r

    def _bfs(self, start, end):
        """BFS 最短路径"""
        if start == end:
            return [start]
        q = deque([(start, [start])])
        visited = {start}
        while q:
            cur, path = q.popleft()
            for nb in self._get_neighbors(*cur):
                if nb == end:
                    return path + [nb]
                if nb not in visited:
                    visited.add(nb)
                    q.append((nb, path + [nb]))
        return None

    @staticmethod
    def _manhattan(a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    # ---- 5种扫描策略 ----
    def _strategy_row_snake(self):
        cells = []
        for y in range(self.HEIGHT):
            col = [(x, y) for x in range(self.WIDTH) if self.grid[x][y] == 0]
            cells.extend(col if y % 2 == 0 else reversed(col))
        return self._align_start(cells)

    def _strategy_column_snake(self):
        cells = []
        for x in range(self.WIDTH):
            row = [(x, y) for y in range(self.HEIGHT) if self.grid[x][y] == 0]
            cells.extend(row if x % 2 == 0 else reversed(row))
        return self._align_start(cells)

    def _strategy_row_snake_rev(self):
        cells = []
        for y in range(self.HEIGHT - 1, -1, -1):
            col = [(x, y) for x in range(self.WIDTH) if self.grid[x][y] == 0]
            cells.extend(col if y % 2 == 0 else reversed(col))
        return self._align_start(cells)

    def _strategy_column_snake_rev(self):
        cells = []
        for x in range(self.WIDTH - 1, -1, -1):
            row = [(x, y) for y in range(self.HEIGHT) if self.grid[x][y] == 0]
            cells.extend(row if x % 2 == 0 else reversed(row))
        return self._align_start(cells)

    def _strategy_nearest(self):
        all_cells = [(x, y) for x in range(self.WIDTH) for y in range(self.HEIGHT) if self.grid[x][y] == 0]
        if self.start_xy not in all_cells:
            return all_cells
        unvisited = set(all_cells)
        order = [self.start_xy]
        unvisited.remove(self.start_xy)
        cur = self.start_xy
        while unvisited:
            nxt = min(unvisited, key=lambda c: self._manhattan(cur, c))
            order.append(nxt)
            unvisited.remove(nxt)
            cur = nxt
        return order

    def _align_start(self, cells):
        if self.start_xy in cells:
            idx = cells.index(self.start_xy)
            if idx > 0:
                cells = cells[idx:] + cells[:idx]
        return cells

    def _order_to_path(self, order):
        """将目标顺序转换为逐格路径"""
        if not order:
            return []
        path = [order[0]]
        cur = order[0]
        for target in order[1:]:
            if cur == target:
                continue
            seg = self._bfs(cur, target)
            if seg is None:
                continue
            for p in seg[1:]:
                path.append(p)
            cur = target
        return path

    # ---- 运行规划 ----
    def plan(self):
        """运行所有策略，选取最优结果"""
        strategies = [
            ("行向蛇形", self._strategy_row_snake),
            ("列向蛇形", self._strategy_column_snake),
            ("反向行向", self._strategy_row_snake_rev),
            ("反向列向", self._strategy_column_snake_rev),
            ("最近邻", self._strategy_nearest),
        ]

        results = []
        for name, func in strategies:
            order = func()
            path = self._order_to_path(order)
            if not path:
                continue
            rp = self._bfs(path[-1], self.start_xy)
            full = path + (rp[1:] if rp and len(rp) > 1 else [])
            # 验证
            all_cells = [(x, y) for x in range(self.WIDTH) for y in range(self.HEIGHT) if self.grid[x][y] == 0]
            covered = set(full)
            missed = [c for c in all_cells if c not in covered]
            invalid = any(abs(full[i][0] - full[i + 1][0]) + abs(full[i][1] - full[i + 1][1]) != 1 for i in range(len(full) - 1))
            forbidden = any(self.grid[p[0]][p[1]] == 1 for p in full)
            if missed or invalid or forbidden:
                continue
            steps = len(full) - 1
            results.append((name, full, steps))

        if not results:
            raise RuntimeError("所有策略均失败！")

        results.sort(key=lambda r: r[2])
        self.best_result = results[0]
        return self.best_result

    # ---- 输出 ----
    def print_report(self):
        """打印详细报告"""
        if not self.best_result:
            print("请先调用 plan()")
            return
        name, full_path, steps = self.best_result

        print("=" * 55)
        print(f"  最优策略: {name}  ({steps}步)")
        print("=" * 55)

        all_cells = [(x, y) for x in range(self.WIDTH) for y in range(self.HEIGHT) if self.grid[x][y] == 0]
        print(f"可通行格子: {len(all_cells)}, 覆盖: {len(set(full_path))}, 航点: {len(full_path)}")
        print(f"验证: 覆盖通过 | 步长通过 | 禁飞通过 | 返航通过")

        # 关键转折点
        kp = [full_path[0]]
        for i in range(1, len(full_path) - 1):
            pd = (full_path[i][0] - full_path[i - 1][0], full_path[i][1] - full_path[i - 1][1])
            nd = (full_path[i + 1][0] - full_path[i][0], full_path[i + 1][1] - full_path[i][1])
            if pd != nd:
                kp.append(full_path[i])
        kp.append(full_path[-1])
        print(f"\n关键转折点 ({len(kp)}个):")
        for p in kp:
            real = self.xy_to_real(p)
            print(f"  {self.xy_to_ab_str(p)}  (内部:{p})  实际:({real[0]:.1f},{real[1]:.1f})")

        # 完整路径
        print("\n" + "-" * 55)
        print("[完整路径]:")
        for i, xy in enumerate(full_path, 1):
            real = self.xy_to_real(xy)
            m = " <-- 起点" if i == 1 else (" <-- 终点" if i == len(full_path) else "")
            print(f"  {i:3d}: {self.xy_to_ab_str(xy)}  实际=({real[0]:.1f}, {real[1]:.1f}){m}")

        # 可视化
        print("\n" + "-" * 55)
        print("[路径可视化]:")
        pg = [["    "] * self.HEIGHT for _ in range(self.WIDTH)]
        for idx, (x, y) in enumerate(full_path):
            pg[x][y] = f"{idx:3d}"
        for x in range(self.WIDTH):
            for y in range(self.HEIGHT):
                if self.grid[x][y] == 1:
                    pg[x][y] = " ## "
        print("    ", end="")
        for x in range(self.WIDTH):
            print(f" A{9 - x}", end=" ")
        print()
        for y in range(self.HEIGHT):
            print(f" B{y + 1} ", end="")
            for x in range(self.WIDTH):
                print(pg[x][y], end="")
            print()

    def save_path_txt(self, filename="coverage_path.txt"):
        """保存路径文件 (A/B坐标格式)"""
        if not self.best_result:
            raise RuntimeError("请先调用 plan()")
        name, full_path, steps = self.best_result
        with open(filename, "w", encoding="utf-8") as f:
            f.write("9x7 全覆盖路径\n")
            f.write(f"策略: {name}\n")
            f.write(f"禁飞区: {self.forbidden_zones_ab}\n")
            f.write(f"步数: {steps}, 航点: {len(full_path)}\n")
            f.write("=" * 40 + "\n")
            for i, xy in enumerate(full_path, 1):
                real = self.xy_to_real(xy)
                f.write(f"{i:03d}: {self.xy_to_ab_str(xy)}  内部=({xy[0]},{xy[1]})  实际=({real[0]:.1f},{real[1]:.1f})\n")
        print(f"路径已保存到 {filename}")

    def save_router_txt(self, filename="router.txt"):
        """
        保存 router 指令文件
        格式：
          x1,y1,z1
          x2,y2,z2
          ...
        其中 x,y,z 为实际坐标值（米）
        """
        if not self.best_result:
            raise RuntimeError("请先调用 plan()")
        _, full_path, _ = self.best_result
        with open(filename, "w", encoding="utf-8") as f:
            # f.write("x,y,z\n")
            for xy in full_path:
                rx, ry = self.xy_to_real(xy)
                f.write(f"{rx:.1f},{ry:.1f},1.0\n")
            f.write(f"0.0,-1.0,1.0\n")  # 沿-y轴后退1m
            f.write(f"0.0,0.0,0.2\n")  # 斜着回到起点
        print(f"router 指令已保存到 {filename} (共 {len(full_path)+2} 个航点)")


# ============================================================
# 使用示例
# ============================================================
if __name__ == "__main__":
    # ---- 配置禁飞区 (A??, B?? 格式) ----
    FORBIDDEN_ZONES = [
        ("A6", "B3"),
        ("A5", "B3"),
        ("A5", "B4"),
    ]

    print("=" * 55)
    print("  9x7 全覆盖路径规划器")
    print("=" * 55)

    planner = CoveragePlanner(FORBIDDEN_ZONES)
    name, full_path, steps = planner.plan()
    planner.print_report()
    # planner.save_path_txt("drone_control/coverage_path.txt")
    planner.save_router_txt("drone_control/router.txt")

    print("\n" + "=" * 55)
    print("完成!")
