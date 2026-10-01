"""Analyze flight log: split by flight, show key metrics."""
import json
import sys

path = sys.argv[1] if len(sys.argv) > 1 else 'drone_control/basic/flight_data.jsonl'

with open(path) as f:
    events = [json.loads(l) for l in f if l.strip()]

# Find flight boundaries
flights = []
for i, e in enumerate(events):
    if e.get('event') == 'task_start':
        for j in range(i + 1, min(i + 5000, len(events))):
            if events[j].get('state') == 'END':
                flights.append((i, j))
                break
        else:
            flights.append((i, min(i + 5000, len(events))))

print(f'Total flights: {len(flights)}\n')

for fi, (start, end) in enumerate(flights):
    t0 = events[start].get('t', 0)
    print(f'--- Flight {fi + 1} (t={t0:.1f}) ---')

    # States
    states_seen = []
    for e in events[start:end]:
        s = e.get('state')
        if s and (not states_seen or s != states_seen[-1]):
            states_seen.append(s)
    print(f'  States: {" -> ".join(states_seen)}')

    # Heading hold
    for e in events[start:end]:
        fr = e.get('heading_fault_reason')
        if fr:
            print(f'  HEADING FAULT: {fr}')
            break
    else:
        print(f'  Heading hold: OK')

    # Waypoints
    for e in events[start:end]:
        if e.get('event') == 'waypoint_advance':
            p = e.get('pos', [0, 0, 0])
            xy = (p[0] ** 2 + p[1] ** 2) ** 0.5
            print(f'  WP {e["target_idx"]}: {e["reason"]}  pos=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.2f})  XY={xy * 100:.0f}cm')

    # LAND
    land_events = [e for e in events[start:end] if e.get('state') == 'LAND']
    if land_events:
        last_land = land_events[-1]
        p = last_land.get('pos', [0, 0, 0])
        xy = (p[0] ** 2 + p[1] ** 2) ** 0.5
        print(f'  LAND: ({p[0]:+.3f},{p[1]:+.3f})  XY={xy * 100:.0f}cm  unlock={last_land.get("unlock_sta")}  pwm={last_land.get("motor_pwm_mask")}')

    # Duration
    for e in events[start:end]:
        if e.get('state') == 'END':
            dur = e.get('t', t0) - t0
            print(f'  Duration: {dur:.0f}s')
            break
    print()
