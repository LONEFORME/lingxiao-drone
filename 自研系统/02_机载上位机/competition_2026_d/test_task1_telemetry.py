from shared.competition_2026_d_protocol import (
    Flag,
    MessageType,
    UavPhase,
    unpack_payload,
)

try:
    from drone_control.competition_2026_d.task1_mission import Task1Phase
    from drone_control.competition_2026_d.task1_telemetry import (
        Task1TelemetryPublisher,
        Task1TelemetrySample,
    )
except ModuleNotFoundError:
    from competition_2026_d.task1_mission import Task1Phase
    from competition_2026_d.task1_telemetry import (
        Task1TelemetryPublisher,
        Task1TelemetrySample,
    )


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeLink:
    def __init__(self):
        self.published = []

    def publish(self, message_type, payload, **kwargs):
        self.published.append((MessageType(message_type), payload, kwargs))
        return len(self.published) - 1


def sample(*, phase=Task1Phase.TAKEOFF, base_state="NAVIGATE"):
    return Task1TelemetrySample(
        phase=phase,
        base_state=base_state,
        position_xyz_m=(1.25, -0.5, 1.5),
    )


def test_uav_state_is_minimal_and_10hz():
    clock = Clock()
    link = FakeLink()
    publisher = Task1TelemetryPublisher(link, session_id=20, clock=clock)
    assert publisher.update(sample())
    state = next(
        item for item in link.published if item[0] == MessageType.UAV_STATE
    )
    assert unpack_payload(MessageType.UAV_STATE, state[1]) == (
        1250,
        -500,
        1500,
    )

    clock.now = 0.05
    assert not publisher.update(sample())
    clock.now = 0.10
    assert publisher.update(sample())
    states = [
        item for item in link.published if item[0] == MessageType.UAV_STATE
    ]
    assert len(states) == 2


def test_phase_events_cover_takeoff_follow_drop_land_and_completion():
    clock = Clock()
    link = FakeLink()
    publisher = Task1TelemetryPublisher(link, session_id=20, clock=clock)
    phases = (
        (Task1Phase.TAKEOFF, UavPhase.TAKEOFF),
        (Task1Phase.FOLLOW_B_C, UavPhase.FORMATION_FOLLOW),
        (Task1Phase.RELEASING, UavPhase.DROP),
        (Task1Phase.LAND_H, UavPhase.LAND_H),
    )
    for index, (task_phase, expected) in enumerate(phases):
        clock.now = float(index)
        assert publisher.update(sample(phase=task_phase))
        event = [
            item
            for item in link.published
            if item[0] == MessageType.UAV_EVENT
        ][-1]
        assert unpack_payload(MessageType.UAV_EVENT, event[1]) == (
            expected,
            index * 1000,
        )
        assert event[2]["flags"] == Flag.ACK_REQUIRED | Flag.EVENT

    assert publisher.finish(mission_success=True, faulted=False)
    assert publisher.finish(mission_success=True, faulted=False)
    events = [
        item for item in link.published if item[0] == MessageType.UAV_EVENT
    ]
    assert sum(
        unpack_payload(MessageType.UAV_EVENT, item[1])[0]
        == UavPhase.COMPLETE
        for item in events
    ) == 1


def test_faulted_finish_sends_no_fault_or_complete_event():
    link = FakeLink()
    publisher = Task1TelemetryPublisher(link, session_id=20)
    assert not publisher.finish(mission_success=False, faulted=True)
    assert link.published == []
