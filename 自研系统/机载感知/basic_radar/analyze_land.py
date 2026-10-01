import json

recs = []
with open("flight_data.jsonl") as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        try:
            d = json.loads(line)
        except Exception:
            continue
        if d.get("state") == "LAND":
            recs.append(d)

t0 = recs[0]["t"]
for d in recs:
    print(f"t={d['t']-t0:6.2f} z={d['pos'][2]:.3f} yaw={d['t265_yaw_deg']:+.2f} "
          f"unlock={d['unlock_sta']} pwm={d['motor_pwm_mask']}")
