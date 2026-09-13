"""RDK板端VS1串口最小检查；不打开飞控、不解锁。"""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from .vision.cybercam_reader import CyberCamReader
from .vision.platform_observation import FeatureFlag


def main(argv=None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyS7")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--require-frames", type=int, default=1)
    parser.add_argument("--output-jsonl", type=Path, default=None)
    args = parser.parse_args(argv)
    reader = CyberCamReader(args.port, args.baudrate)
    if not reader.start():
        print(json.dumps({"ok": False, "reason": "open_failed"}))
        return 2
    output = None
    recorded = 0
    found = 0
    apriltag_valid = 0
    last_key = None
    try:
        if args.output_jsonl is not None:
            args.output_jsonl.parent.mkdir(parents=True, exist_ok=True)
            output = args.output_jsonl.open("w", encoding="utf-8", buffering=1)
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            now = time.monotonic()
            observation = reader.latest(now, max(1.0, args.duration + 1.0))
            if observation is not None:
                key = (observation.stream_id, observation.seq)
                if key != last_key:
                    last_key = key
                    recorded += 1
                    found += int(observation.found)
                    flags = FeatureFlag(observation.flags)
                    apriltag_valid += int(bool(flags & FeatureFlag.APRILTAG_VALID))
                    if output is not None:
                        output.write(json.dumps({
                            "received_monotonic": observation.received_monotonic,
                            "stream_id": observation.stream_id,
                            "seq": observation.seq,
                            "capture_ms": observation.capture_ms,
                            "found": observation.found,
                            "cx": observation.cx,
                            "cy": observation.cy,
                            "outer_px": observation.outer_px,
                            "inner_px": observation.inner_px,
                            "angle_cdeg": observation.angle_cdeg,
                            "quality": observation.quality,
                            "flags": observation.flags,
                        }, ensure_ascii=False) + "\n")
            time.sleep(0.005)
        stats = reader.stats()
        latest = reader.latest(time.monotonic(), max(1.0, args.duration + 1.0))
        result = {
            "ok": stats["accepted_frames"] >= args.require_frames,
            **stats,
            "recorded_frames": recorded,
            "found_frames": found,
            "apriltag_valid_frames": apriltag_valid,
            "latest": None if latest is None else {
                "stream_id": latest.stream_id,
                "seq": latest.seq,
                "cx": latest.cx,
                "cy": latest.cy,
                "quality": latest.quality,
                "flags": latest.flags,
            },
        }
        print(json.dumps(result, ensure_ascii=False))
        return 0 if result["ok"] else 1
    finally:
        if output is not None:
            output.close()
        reader.close()


if __name__ == "__main__":
    raise SystemExit(main())
