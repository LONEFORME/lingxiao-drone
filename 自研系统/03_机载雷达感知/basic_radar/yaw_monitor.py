"""问题16 yaw测试实时监控：tail flight_data.jsonl，节流到约1行/秒，
并在检测到"同向持续漂移超过阈值"时立刻打印ALERT(不受节流限制)。
不修改任何飞行控制逻辑，纯只读监控脚本。"""
import json
import sys
import time
from collections import deque

LOG_PATH = "flight_data.jsonl"
ALERT_WINDOW_S = 3.0
ALERT_DEG = 10.0
PRINT_INTERVAL_S = 1.0

history = deque()  # (t, yaw_deg)
last_print = 0.0
last_state = None


def follow(path):
    with open(path, "r") as f:
        f.seek(0, 2)
        while True:
            line = f.readline()
            if not line:
                time.sleep(0.05)
                continue
            yield line


for line in follow(LOG_PATH):
    line = line.strip()
    if not line:
        continue
    try:
        d = json.loads(line)
    except Exception:
        continue

    if "event" in d:
        print(f"EVENT {d}", flush=True)
        continue

    t = d.get("t")
    yaw = d.get("t265_yaw_deg")
    vyaw = d.get("vyaw")
    state = d.get("state")
    if t is None or yaw is None:
        continue

    if state != last_state:
        print(f"STATE_CHANGE -> {state} @ t={t} yaw={yaw}", flush=True)
        last_state = state

    history.append((t, yaw))
    while history and t - history[0][0] > ALERT_WINDOW_S:
        history.popleft()

    if len(history) >= 2:
        t0, yaw0 = history[0]
        delta = yaw - yaw0
        if abs(delta) >= ALERT_DEG:
            print(f"ALERT yaw同向漂移 {delta:+.1f}度 在 {t - t0:.1f}秒内 "
                  f"(t={t}, yaw={yaw}, vyaw={vyaw}) —— 建议立即接管", flush=True)

    if t - last_print >= PRINT_INTERVAL_S:
        print(f"t={t:.1f} state={state} yaw={yaw:+.1f} vyaw={vyaw}", flush=True)
        last_print = t
