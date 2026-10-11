"""
K230 实机通信测试 v2
===================
协议帧 (二进制, AA头+FF尾):
  Pi→K230: AA 10 grid_idx FF                       — 启动检测(4B)
  K230→Pi: AA 20 grid_idx cls cnt total conf FF    — 检测结果(8B)
  Pi→K230: AA 11 grid_idx FF                       — ACK确认(4B)

用法:
  python test_k230_live.py          # 交互模式
  python test_k230_live.py 0 0      # 测试格子 (A9,B1)
  python test_k230_live.py 3 2      # 测试格子 (A6,B3)

K230 内部: 30帧累积 → 占比>=70% ∧ 置信>=50% → 通过发RESULT
          否则 → 自动重测30帧（最多1次）→ 强制发RESULT
"""

from Lcode.k230_client import K230Client
import time
import sys

ANIMALS = ["tiger", "wolf", "monkey", "peacock", "elephant"]

# 测试统计
_results = []


def test_single_grid(k230, ix, iy):
    """单格检测: 发 START -> 等 RESULT -> 发 ACK"""
    grid_idx = iy * 9 + ix
    k230.send_start(grid_idx)
    t0 = time.time()
    print(f"Sent START -> grid ({ix},{iy}) idx={grid_idx}")

    for _ in range(150):  # 5s timeout @ ~33ms
        r = k230.poll_result()
        # print(r)
        if r:
            gidx, cls_id, best_cnt, total_dets, avg_conf = r
            elapsed = time.time() - t0

            label = ANIMALS[cls_id] if cls_id < 5 else "NO_ANIMAL"
            if best_cnt > 0:
                dominance = best_cnt / max(total_dets, 1)
                print(f"Got RESULT <- grid={gidx} cls={cls_id}({label}) "
                      f"cnt={best_cnt}/{total_dets} dom={dominance:.0%} conf={avg_conf}% "
                      f"({elapsed:.1f}s)")
            else:
                print(f"Got RESULT <- grid={gidx} cls={cls_id}({label}) "
                      f"empty ({elapsed:.1f}s)")

            k230.send_ack(grid_idx)
            _results.append((ix, iy, cls_id, best_cnt, avg_conf))
            return r
        time.sleep(0.03)

    print(f"Timeout: no result from K230 after {time.time()-t0:.1f}s")
    _results.append((ix, iy, None, 0, 0))
    return None


def interactive(k230):
    """交互模式: 手动输入格子坐标"""
    print("K230 internal eval test  (q=quit, s=stats)")
    print("  grid: 0,0=(A9,B1)  3,2=(A6,B3)  8,6=(A1,B7)")
    while True:
        try:
            inp = input("\nix,iy > ").strip()
            if inp.lower() == 'q':
                break
            if inp.lower() == 's':
                print_stats()
                continue
            parts = [p.strip() for p in inp.split(',')]
            if len(parts) != 2:
                print("Format: ix,iy")
                continue
            ix, iy = int(parts[0]), int(parts[1])
            if not (0 <= ix < 9 and 0 <= iy < 7):
                print("Range: ix 0-8, iy 0-6")
                continue
            test_single_grid(k230, ix, iy)
        except (ValueError, KeyboardInterrupt):
            break


def print_stats():
    print(f"\n--- Test Stats ({len(_results)} grids) ---")
    for ix, iy, cls_id, best_cnt, avg_conf in _results:
        label = ANIMALS[cls_id] if cls_id is not None and cls_id < 5 else "?"
        print(f"  ({ix},{iy}): {label} cnt={best_cnt} conf={avg_conf}%")
    print()


if __name__ == "__main__":
    k230 = K230Client("/dev/ttyS3", 115200)

    if len(sys.argv) == 3:
        try:
            ix, iy = int(sys.argv[1]), int(sys.argv[2])
            test_single_grid(k230, ix, iy)
        except ValueError:
            interactive(k230)
    else:
        interactive(k230)

    k230.close()
    print("Done.")
