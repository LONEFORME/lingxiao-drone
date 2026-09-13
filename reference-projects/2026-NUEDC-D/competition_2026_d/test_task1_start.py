import json
import struct
from pathlib import Path

from shared.competition_2026_d_protocol import (
    CarStateFlag,
    Device,
    Flag,
    Frame,
    MessageType,
    PositionFlag,
    pack_payload,
)

try:
    from drone_control.competition_2026_d.task1_start import (
        Task1StartGate,
        build_joint_drop_test_config,
        build_joint_path_drop_test_config,
        _load_task_config,
        _wait_for_vision,
    )
except ModuleNotFoundError:
    from competition_2026_d.task1_start import (
        Task1StartGate,
        build_joint_drop_test_config,
        build_joint_path_drop_test_config,
        _load_task_config,
        _wait_for_vision,
    )


class FakeLink:
    def __init__(self):
        self.callbacks = []
        self.published = []
        self.acks = []

    def add_callback(self, callback):
        self.callbacks.append(callback)

    def publish(self, message_type, payload, **kwargs):
        self.published.append((message_type, payload, kwargs))
        return len(self.published)

    def acknowledge(self, frame, result=0):
        self.acks.append((frame, result))

    def feed(self, frame):
        for callback in self.callbacks:
            callback(frame)


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeVisionReader:
    def __init__(self, stats):
        self._stats = stats

    def stats(self):
        return dict(self._stats)


def test_vision_startup_wait_accepts_ready_duplex_link():
    reader = FakeVisionReader({
        "running": True,
        "accepted_frames": 3,
        "pongs_received": 1,
        "ping_write_errors": 0,
    })

    assert _wait_for_vision(reader, timeout_s=0.1)


def test_vision_startup_wait_stops_when_reader_thread_dies():
    reader = FakeVisionReader({
        "running": False,
        "accepted_frames": 0,
        "pongs_received": 0,
        "ping_write_errors": 1,
    })

    assert not _wait_for_vision(reader, timeout_s=1.0)


def test_formal_task1_config_loads_validated_curve_and_straight_speeds():
    config_path = Path(__file__).with_name("config.json")
    data = json.loads(config_path.read_text(encoding="utf-8"))

    config = _load_task_config(data)

    assert config.curve_speed_m_s == 0.06
    assert config.car_speed_m_s == 0.075
    assert config.path_lookahead_m == 0.12
    assert config.drop_during_bc_enabled
    assert config.drop_at_follow_height
    assert config.drop_max_error_m == 0.15
    assert config.drop_confirm_duration_s == 0.40
    assert config.min_follow_before_drop_s == 0.0
    assert config.vision_trim_deadband_m == 0.05
    assert config.vision_trim_max_speed_m_s == 0.03
    assert config.release_timeout_s == 2.0
    assert config.hold_duration_s == 2.0
    assert config.takeoff_ascent_slew_m_s == 0.40
    assert config.acquire_vision_min_quality == 40
    assert config.acquire_vision_max_speed_m_s == 0.12
    assert config.acquire_vision_control_period_s == 0.20
    assert config.acquire_vision_filter_window_s == 0.60
    assert config.vision_takeover_max_speed_m_s == 0.08
    assert config.intercept_vision_early_stop_enabled
    assert config.final_landing_radius_m == 0.08
    assert config.final_landing_max_speed_m_s == 0.05
    assert config.final_landing_stable_s == 0.50
    assert config.final_descend_horizontal_max_speed_m_s == 0.06


def test_joint_drop_test_overrides_only_runtime_mission_parameters():
    config = build_joint_drop_test_config(_load_task_config(
        json.loads(
            Path(__file__).with_name("config.json").read_text(encoding="utf-8")
        )
    ))

    assert config.cruise_height_m == 1.2
    assert config.follow_height_m == 1.2
    assert config.hold_duration_s == 3.0
    assert config.car_speed_m_s == 0.05
    assert config.curve_speed_m_s == 0.05
    assert config.drop_max_error_m == 0.20
    assert config.drop_confirm_duration_s == 3.0
    assert config.payload_drop_enabled
    assert config.drop_during_bc_enabled
    assert config.drop_at_follow_height
    assert config.vision_track_only


def test_joint_path_drop_test_waits_at_b_pre_then_uses_constant_speed():
    config = build_joint_path_drop_test_config(
        _load_task_config(
            json.loads(
                Path(__file__).with_name("config.json").read_text(
                    encoding="utf-8"
                )
            )
        ),
        follow_speed_m_s=0.08,
        follow_duration_s=3.5,
    )

    assert config.cruise_height_m == 1.2
    assert config.follow_height_m == 1.2
    assert config.car_speed_m_s == 0.08
    assert config.curve_speed_m_s == 0.08
    assert config.min_follow_before_drop_s == 3.5
    assert config.drop_confirm_duration_s == 0.20
    assert config.drop_max_error_m == 0.30
    assert config.vision_trim_max_speed_m_s == 0.0
    assert config.payload_drop_enabled
    assert config.drop_during_bc_enabled
    assert config.drop_at_follow_height
    assert not config.vision_track_only
    assert not config.intercept_vision_early_stop_enabled


def car_start(*, mode=1, session=10, flags=None):
    return Frame(
        message_type=MessageType.CAR_START,
        flags=int(
            Flag.ACK_REQUIRED | Flag.EVENT if flags is None else flags
        ),
        source=Device.CAR,
        dest=Device.UAV,
        session_id=session,
        seq=1,
        sender_ms=1,
        payload=pack_payload(MessageType.CAR_START, (mode, 1234)),
    )


def car_state(
    *,
    session=20,
    seq=2,
    segment=1,
    track_s_mm=100,
    speed_mm_s=130,
    heading_cdeg=0,
    state_flags=int(
        CarStateFlag.NORMAL_TRACKING | CarStateFlag.ENCODER_SPEED_VALID
    ),
    vx_mm_s=130,
    vy_mm_s=0,
    legacy=False,
):
    values = (
        segment,
        track_s_mm,
        speed_mm_s,
        heading_cdeg,
        state_flags,
    )
    payload = (
        struct.pack("<BHhhH", *values)
        if legacy
        else pack_payload(
            MessageType.CAR_STATE,
            (*values, vx_mm_s, vy_mm_s),
        )
    )
    return Frame(
        message_type=MessageType.CAR_STATE,
        flags=Flag.NONE,
        source=Device.CAR,
        dest=Device.UAV,
        session_id=session,
        seq=seq,
        sender_ms=seq * 100,
        payload=payload,
    )


def car_position(
    *,
    session=0,
    seq=1,
    x_mm=1500,
    y_mm=2000,
    age_ms=25,
    flags=int(
        PositionFlag.CAR_POSE_VALID | PositionFlag.CAR_POSE_FRESH
    ),
):
    return Frame(
        message_type=MessageType.CAR_POSITION,
        flags=Flag.NONE,
        source=Device.CAR,
        dest=Device.UAV,
        session_id=session,
        seq=seq,
        sender_ms=seq * 100,
        payload=pack_payload(
            MessageType.CAR_POSITION,
            (x_mm, y_mm, age_ms, flags),
        ),
    )


def test_gate_rejects_wrong_mode_and_accepts_task1_once():
    link = FakeLink()
    gate = Task1StartGate(link, config_hash=99)
    link.feed(car_start(mode=2))
    assert gate.session_id is None
    link.feed(car_start(session=20))
    link.feed(car_start(session=21))
    assert gate.session_id == 20
    assert gate.car_config_hash == 1234
    assert [result for _frame, result in link.acks] == [1, 0, 1]


def test_gate_requires_event_and_ack_flags():
    link = FakeLink()
    gate = Task1StartGate(link, config_hash=99)
    link.feed(car_start(flags=Flag.EVENT))
    assert gate.session_id is None
    assert gate.rejected_start_frames == 1
    assert link.acks == []


def test_gate_replies_negative_ack_to_ack_required_invalid_start():
    link = FakeLink()
    gate = Task1StartGate(link, config_hash=99)
    frame = car_start(flags=Flag.ACK_REQUIRED)
    link.feed(frame)
    assert gate.session_id is None
    assert link.acks == [(frame, 1)]


def test_car_speed_requires_bound_session_and_fresh_data():
    clock = Clock()
    link = FakeLink()
    gate = Task1StartGate(link, config_hash=99, clock=clock)
    link.feed(car_start(session=20))
    link.feed(car_state())
    assert gate.car_speed() == 0.13
    assert gate.car_velocity() == (0.13, 0.0)
    clock.now = 0.31
    assert gate.car_speed() is None
    assert gate.car_velocity() is None


def test_car_state_rejects_wrong_session_invalid_flag_and_reserved_bits():
    link = FakeLink()
    gate = Task1StartGate(link, config_hash=99)
    link.feed(car_start(session=20))
    link.feed(car_state(session=21))
    link.feed(car_state(seq=3, state_flags=0))
    assert gate.latest_car_state() is not None
    assert gate.car_speed() is None
    link.feed(car_state(seq=4, state_flags=0x0020))
    assert gate.latest_car_state().seq == 3
    assert gate.rejected_state_frames == 2


def test_car_state_sequence_rejects_duplicate_and_old_but_allows_wrap():
    link = FakeLink()
    gate = Task1StartGate(link, config_hash=99)
    link.feed(car_start(session=20))
    link.feed(car_state(seq=65535))
    link.feed(car_state(seq=65535, speed_mm_s=120, vx_mm_s=120))
    link.feed(car_state(seq=65534, speed_mm_s=110, vx_mm_s=110))
    assert gate.latest_car_state().seq == 65535
    link.feed(car_state(seq=0, speed_mm_s=140, vx_mm_s=140))
    assert gate.latest_car_state().seq == 0
    assert gate.car_speed() == 0.14
    assert gate.duplicate_or_old_state_frames == 2


def test_legacy_state_keeps_scalar_but_has_no_world_velocity():
    link = FakeLink()
    gate = Task1StartGate(link, config_hash=99)
    link.feed(car_start(session=20))
    link.feed(car_state(legacy=True))
    state = gate.latest_car_state()
    assert state.legacy_payload
    assert state.world_velocity_m_s is None
    assert gate.car_speed() == 0.13
    assert gate.car_velocity() is None
    assert gate.legacy_state_frames == 1


def test_signed_reverse_speed_is_saved_but_not_used_for_task1_forward_speed():
    link = FakeLink()
    gate = Task1StartGate(link, config_hash=99)
    link.feed(car_start(session=20))
    link.feed(
        car_state(speed_mm_s=-100, vx_mm_s=-100, vy_mm_s=0)
    )
    assert gate.latest_car_state().speed_m_s == -0.1
    assert gate.car_speed() is None
    assert gate.car_velocity() == (-0.1, 0.0)


def test_car_world_velocity_is_preserved_without_coordinate_conversion():
    link = FakeLink()
    gate = Task1StartGate(link, config_hash=99)
    link.feed(car_start(session=20))
    link.feed(car_state(vx_mm_s=100, vy_mm_s=-50))
    assert gate.car_velocity() == (0.1, -0.05)


def test_car_position_accepts_session_zero_before_start_then_bound_session():
    clock = Clock()
    link = FakeLink()
    gate = Task1StartGate(link, config_hash=99, clock=clock)
    link.feed(car_position())
    assert gate.latest_car_position().position_xy_m == (1.5, 2.0)

    link.feed(car_start(session=20))
    assert gate.latest_car_position() is None
    link.feed(
        car_position(
            session=20,
            seq=0,
            x_mm=1510,
            flags=int(
                PositionFlag.CAR_POSE_VALID
                | PositionFlag.CAR_POSE_FRESH
                | PositionFlag.SESSION_VALID
            ),
        )
    )
    assert gate.latest_car_position().position_xy_m == (1.51, 2.0)
    clock.now = 0.31
    assert gate.latest_car_position() is None


def test_car_position_rejects_session_and_reserved_flag_errors():
    link = FakeLink()
    gate = Task1StartGate(link, config_hash=99)
    link.feed(car_position(session=9))
    link.feed(car_position(seq=2, flags=0x0010))
    assert gate.latest_car_position() is None
    assert gate.rejected_position_frames == 2
