"""
T-016 测试结果分析脚本
分析三次飞行日志，对照通过标准输出结果
"""
import json
import sys
import os


def analyze(path):
    with open(path) as f:
        events = [json.loads(l) for l in f if l.strip()]

    # 三段飞行：三个 TAKEOFF 起始位置（在 LAND 之后的 TAKEOFF）
    takeoff_starts = []
    for i, e in enumerate(events):
        if e.get('state') == 'TAKEOFF':
            if not takeoff_starts:
                takeoff_starts.append(i)
            else:
                for j in range(takeoff_starts[-1], i):
                    if events[j].get('state') == 'LAND':
                        takeoff_starts.append(i)
                        break

    flight_ends = []
    for i in range(len(takeoff_starts)):
        if i + 1 < len(takeoff_starts):
            flight_ends.append(takeoff_starts[i+1] - 1)
        else:
            flight_ends.append(len(events) - 1)

    test_info = [
        ('A: 长路径（2m×2m矩形，评估漂移累积）',
         [(0.0, 0.0, 1.0), (2.0, 0.0, 1.0), (2.0, 2.0, 1.0),
          (0.0, 2.0, 1.0), (0.0, 0.0, 1.0), (0.0, 0.0, 0.15)]),
        ('B: 短路径（0.5m正方形，评估低空精度）',
         [(0.0, 0.0, 0.5), (0.5, 0.0, 0.5), (0.5, 0.5, 0.5),
          (0.0, 0.5, 0.5), (0.0, 0.0, 0.5), (0.0, 0.0, 0.15)]),
        ('C: 高度变化（0.5/1.0/1.5m，评估垂直通道）',
         [(0.0, 0.0, 0.5), (0.0, 0.0, 1.0), (0.0, 0.0, 1.5),
          (0.0, 0.0, 1.0), (0.0, 0.0, 0.5), (0.0, 0.0, 0.15)]),
    ]

    all_pass = True
    results = []

    for fi, (start, end) in enumerate(zip(takeoff_starts, flight_ends)):
        seg = events[start:end+1]
        name, targets = test_info[fi]
        wps = [e for e in seg if e.get('event') == 'waypoint_advance']

        print()
        print('=' * 60)
        print(f'  {name}')
        print('=' * 60)
        print(f'  航点 ({len(wps)}):')

        flight_pass = True
        flight_notes = []

        for wi, wp in enumerate(wps):
            p = wp.get('pos', [0, 0, 0])
            t = targets[wi] if wi < len(targets) else (0, 0, 0)
            tx, ty, tz = t
            xy_err = ((p[0] - tx)**2 + (p[1] - ty)**2) ** 0.5
            z_err = abs(p[2] - tz)
            reason = wp.get('reason', '')

            # 通过阈值
            if fi == 0 and wi == 4:      # 长路径回到原点
                threshold = 0.15
                label = '回到原点'
            elif wi == 5:                 # 降落点
                threshold = 0.10
                label = '降落点'
            elif fi == 0:                 # 长路径其他航点
                threshold = 0.15
                label = ''
            else:                         # 短路径/高度变化
                threshold = 0.10
                label = ''

            ok = xy_err <= threshold
            if not ok:
                flight_pass = False
            icon = '✅' if ok else '❌'
            tag = f'({label})' if label else ''
            status = '通过' if ok else f'超标({threshold*100:.0f}cm)'

            print(f'  {icon} WP{wi}: {reason:20s} '
                  f'目标=({tx:+4.1f},{ty:+4.1f},{tz:.1f}) '
                  f'实际=({p[0]:+7.3f},{p[1]:+7.3f},{p[2]:+.2f}) '
                  f'XY={xy_err*100:3.0f}cm{tag:12s} {status}')

        # 航向
        hf = None
        for e in seg:
            fr = e.get('heading_fault_reason')
            if fr:
                hf = fr
                break
        hf_ok = hf is None
        if not hf_ok:
            flight_pass = False
            all_pass = False
        print(f'  {"✅" if hf_ok else "❌"} 航向保持: {"正常" if hf_ok else f"故障: {hf}"}')

        # Yaw
        first_yaw = last_yaw = None
        for e in seg:
            y = e.get('t265_yaw_deg')
            if y is not None:
                if first_yaw is None:
                    first_yaw = y
                last_yaw = y
        if first_yaw is not None and last_yaw is not None:
            yaw_drift = abs(last_yaw - first_yaw)
            yaw_ok = yaw_drift < 5.0
            if not yaw_ok:
                flight_pass = False
            print(f'  {"✅" if yaw_ok else "❌"} Yaw漂移: {last_yaw-first_yaw:+.2f}° (阈值5°)')

        # 降落锁桨
        land = [e for e in seg if e.get('state') == 'LAND']
        if land:
            last = land[-1]
            p = last.get('pos', [0, 0, 0])
            us = last.get('unlock_sta', '?')
            mp = last.get('motor_pwm_mask', '?')
            locked = (us == 0 and mp == 0)
            if not locked:
                flight_pass = False
            print(f'  {"✅" if locked else "❌"} 降落锁桨: unlock={us} motor_pwm={mp} '
                  f'pos=({p[0]:+.3f},{p[1]:+.3f})')

        if not flight_pass:
            all_pass = False

        results.append((name, flight_pass))
        print()

    # 汇总
    print('=' * 60)
    print(f'  T-016 测试结果汇总')
    print('=' * 60)
    for name, passed in results:
        print(f'  {"✅" if passed else "❌"} {name}')
    print(f'  {"✅ 全部通过！" if all_pass else "❌ 存在未通过项"}')
    print('=' * 60)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('用法: python tools/analyze_t016.py <flight_data.jsonl>')
        sys.exit(1)
    analyze(sys.argv[1])
