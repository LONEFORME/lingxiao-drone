"""2026 competition-preparation entry point.

Examples::

    python competition_main.py --phase scout --dry-plan
    python competition_main.py --phase execute --points P2,P5 --dry-plan
    python competition_main.py --phase scout
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Sequence

from Lcode.competition_plan import (
    CompetitionPlanError,
    load_competition_config,
    plan_mission,
)
from Lcode.mission_events import MissionEventBus
from Lcode.mission_session import MissionSession, MissionSessionError
from Lcode.video_source import (
    VideoSourceError,
    create_video_source,
    load_video_catalog,
)
from Lcode.waypoint_snapshot import (
    SnapshotPolicyError,
    WaypointSnapshotConsumer,
    load_snapshot_policy,
)


DEFAULT_CONFIG = Path(__file__).with_name("competition_config.json")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="2026 competition mission runner")
    parser.add_argument("--phase", choices=("scout", "execute"), required=True)
    parser.add_argument(
        "--points",
        default="",
        help="Comma-separated point ids for execute phase, for example P2,P5",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument(
        "--session",
        type=Path,
        help="Reuse a session directory, normally the directory created by scout",
    )
    parser.add_argument(
        "--sessions-root",
        type=Path,
        default=Path(__file__).with_name("sessions"),
        help="Directory used when creating a new session",
    )
    parser.add_argument(
        "--dry-plan",
        action="store_true",
        help="Print the generated mission without initializing flight hardware",
    )
    return parser


def parse_point_ids(raw: str) -> tuple[str, ...]:
    return tuple(part.strip() for part in raw.split(",") if part.strip())


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = load_competition_config(args.config)
        planned = plan_mission(config, args.phase, parse_point_ids(args.points))
        video_catalog = load_video_catalog(args.config)
        snapshot_policy = load_snapshot_policy(args.config)
    except (CompetitionPlanError, VideoSourceError, SnapshotPolicyError) as exc:
        print(f"Mission planning failed: {exc}")
        return 2

    preview = planned.as_dict()
    preview["video"] = video_catalog.as_dict()
    preview["auto_snapshot"] = snapshot_policy.as_dict()
    print(json.dumps(preview, ensure_ascii=False, indent=2))
    if args.dry_plan:
        return 0

    try:
        session = MissionSession.create(args.sessions_root, args.session)
        run_id = session.begin(config.name, planned.phase, preview)
    except (MissionSessionError, OSError) as exc:
        print(f"Session setup failed: {exc}")
        return 2

    event_bus = MissionEventBus()
    event_bus.subscribe(session.record_event)
    snapshot_consumer = None
    snapshot_setup_error = None

    if snapshot_policy.enabled:
        try:
            if video_catalog.active is None:
                raise VideoSourceError(
                    "auto_snapshot is enabled but video.active_profile is none"
                )
            video_source = create_video_source(video_catalog.active.receiver)
            snapshot_consumer = WaypointSnapshotConsumer(
                source=video_source,
                policy=snapshot_policy,
                output_dir=session.snapshots_dir,
                run_id=run_id,
                result_sink=event_bus.publish,
            )
            if not snapshot_consumer.start():
                snapshot_setup_error = snapshot_consumer.last_error or "unknown_error"
                snapshot_consumer.stop()
                if snapshot_policy.required or snapshot_consumer.startup_timed_out:
                    session.finish(
                        planned.phase,
                        "setup_failed",
                        snapshot_setup_error=snapshot_setup_error,
                        auto_snapshot=snapshot_consumer.stats(),
                    )
                    print(f"Snapshot setup failed: {snapshot_setup_error}")
                    return 2
                print(f"Snapshot disabled after setup failure: {snapshot_setup_error}")
                snapshot_consumer = None
            else:
                event_bus.subscribe(snapshot_consumer.handle_event)
        except (OSError, VideoSourceError, ValueError) as exc:
            snapshot_setup_error = str(exc)
            if snapshot_policy.required:
                session.finish(
                    planned.phase,
                    "setup_failed",
                    snapshot_setup_error=snapshot_setup_error,
                )
                print(f"Snapshot setup failed: {snapshot_setup_error}")
                return 2
            print(f"Snapshot disabled after setup failure: {snapshot_setup_error}")

    event_bus.start()
    print(f"Session directory: {session.path}")

    # Importing the hardware entry point is intentionally delayed so route
    # planning can be tested on a development PC without RealSense/GPIO.
    from main import main as run_basic_flight

    flight_status = "finished"
    try:
        run_basic_flight(
            targets=[list(point) for point in planned.waypoints],
            waypoint_holds=list(planned.hold_s),
            point_ids=list(planned.point_ids),
            waypoint_actions=list(planned.actions),
            event_sink=event_bus.publish,
        )
    except BaseException:
        flight_status = "interrupted"
        raise
    finally:
        if snapshot_consumer is not None:
            snapshot_consumer.stop()
        event_bus.close()
        result_details = {
            "dropped_events": event_bus.dropped_events,
            "snapshot_setup_error": snapshot_setup_error,
        }
        if snapshot_consumer is not None:
            result_details["auto_snapshot"] = snapshot_consumer.stats()
        try:
            session.finish(planned.phase, flight_status, **result_details)
        except (MissionSessionError, OSError) as exc:
            print(f"Session finalization failed: {exc}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

