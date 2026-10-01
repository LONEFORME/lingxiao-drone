"""
飞行日志分析工具 — 快速分析 flight_data.jsonl

用法:
  python tools/flight_log_analyzer.py                              # 默认分析 basic 目录
  python tools/flight_log_analyzer.py path/to/flight_data.jsonl    # 指定文件

输出:
  - 每次飞行的状态机流转
  - 航点到达精度 (XY 偏差)
  - 航向保持状态
  - 降落点位置 + 锁桨确认
  - 快速摘要 (适合复制到聊天记录)
"""
import json
import os
import sys
from collections import defaultdict


def load_events(path):
    with open(path, encoding='utf-8') as f:
        return [json.loads(l) for l in f if l.strip()]


def find_flights(events):
    """按 task_start → END 分割为独立的飞行。"""
    flights = []
    for i, e in enumerate(events):
        if e.get('event') != 'task_start':
            continue
        # 找本次飞行的 END
        for j in range(i + 1, min(i + 10000, len(events))):
            if events[j].get('state') == 'END':
                flights.append((i, j))
                break
        else:
            flights.append((i, min(i + 10000, len(events))))
    return flights


def heading_drift_deg(events):
    """取飞行中 yaw 的变化量：最后一个有效值减第一个。"""
    first_yaw = None
    last_yaw = None
    for e in events:
        y = e.get('t265_yaw_deg')
        if y is not None:
            if first_yaw is None:
                first_yaw = y
            last_yaw = y
    if first_yaw is not None and last_yaw is not None:
        return last_yaw - first_yaw
    return None


def analyze(path):
    events = load_events(path)
    flights = find_flights(events)

    if not flights:
        print("未找到飞行记录（需要 task_start 事件标记飞行开始）")
        return

    print(f"文件: {path}  ({len(events)} 条记录, {len(flights)} 次飞行)\n")

    for fi, (start, end) in enumerate(flights):
        # task_start 事件本身没有 t 字段，取下一个有 t 的事件作为飞行开始时间
        t0 = 0
        for e in events[start:end + 1]:
            t0 = e.get('t', 0)
            if t0:
                break
        flight_slice = events[start:end + 1]

        # 状态序列
        states = []
        for e in flight_slice:
            s = e.get('state')
            if s and (not states or s != states[-1]):
                states.append(s)

        dur = 0
        for e in flight_slice:
            if e.get('state') == 'END':
                dur = e.get('t', t0) - t0
                break

        # 航向保持
        heading_fault = None
        for e in flight_slice:
            fr = e.get('heading_fault_reason')
            if fr:
                heading_fault = fr
                break

        yaw_drift = heading_drift_deg(flight_slice)

        # 航点 + 降落
        waypoints = [e for e in flight_slice if e.get('event') == 'waypoint_advance']
        land_events = [e for e in flight_slice if e.get('state') == 'LAND']

        # --- 输出 ---
        print(f"{'=' * 50}")
        print(f"飞行 {fi + 1}  (t={t0:.1f}, 时长 {dur:.0f}s)")
        print(f"{'=' * 50}")

        print(f"  状态机: {' → '.join(states)}")

        # 航向
        if heading_fault:
            print(f"  ⚠ 航向故障: {heading_fault}")
        else:
            print(f"  ✅ 航向保持: 正常")
        if yaw_drift is not None:
            print(f"     Yaw 变化: {yaw_drift:+.2f}°")

        # 航点
        for wp in waypoints:
            p = wp.get('pos', [0, 0, 0])
            t = wp.get('target', [0, 0, 0])
            xy_err = (p[0] ** 2 + p[1] ** 2) ** 0.5
            z_err = abs(p[2] - t[2]) if len(t) > 2 else 0
            print(f"  航点 {wp['target_idx']}: "
                  f"{wp['reason']:20s}  "
                  f"pos=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.2f})  "
                  f"XY偏差={xy_err * 100:.0f}cm  Z偏差={z_err * 100:.0f}cm")

        # 降落
        if land_events:
            last = land_events[-1]
            p = last.get('pos', [0, 0, 0])
            xy_err = (p[0] ** 2 + p[1] ** 2) ** 0.5
            us = last.get('unlock_sta', '?')
            mp = last.get('motor_pwm_mask', '?')
            yaw = last.get('t265_yaw_deg', '?')
            print(f"  ── 降落 ──")
            print(f"     位置: ({p[0]:+.3f}, {p[1]:+.3f})  XY偏差={xy_err * 100:.0f}cm")
            print(f"     锁桨: unlock={us}  motor_pwm={mp}")
            print(f"     Yaw: {yaw}°")
            locked = (us == 0 and (mp == 0 or mp == '?'))
            print(f"     状态: {'✅ 成功锁桨' if locked else '⚠ 未确认锁桨'}")

        # 快速摘要行
        wp_summary = ', '.join(
            f"WP{w['target_idx']}:{w['reason'][:4]}{{{(w['pos'][0]**2 + w['pos'][1]**2)**0.5 * 100:.0f}cm}}"
            for w in waypoints
        )
        print(f"  摘要: {wp_summary}  |  yaw={yaw_drift:+.1f}°" if yaw_drift is not None else f"  摘要: {wp_summary}")
        print()


def main():
    if len(sys.argv) > 1:
        path = sys.argv[1]
    else:
        # 默认搜索：当前目录 → 项目目录 → 归档数据
        candidates = [
            'flight_data.jsonl',
            'drone_control/basic/flight_data.jsonl',
            os.path.join(os.path.dirname(__file__), '..', 'drone_control', 'basic', 'flight_data.jsonl'),
        ]
        # 也搜索归档目录下最新的 jsonl
        archive_root = os.path.join(os.path.dirname(__file__), '..', 'drone_control', 'tools', 'data_archive')
        if os.path.isdir(archive_root):
            dirs = sorted(os.listdir(archive_root), reverse=True)
            for d in dirs:
                test_dir = os.path.join(archive_root, d)
                if os.path.isdir(test_dir):
                    for f in sorted(os.listdir(test_dir), reverse=True):
                        if f.endswith('.jsonl') and not f.endswith('.bak'):
                            candidates.append(os.path.join(test_dir, f))
        for c in candidates:
            if os.path.exists(c):
                path = c
                break
        else:
            print("未找到 flight_data.jsonl，请指定文件路径")
            sys.exit(1)

    analyze(path)


if __name__ == '__main__':
    main()
