from types import SimpleNamespace

from shared.competition_2026_d_protocol import (
    Device,
    Flag,
    Frame,
    MessageType,
    pack_payload,
    unpack_payload,
)

try:
    from drone_control.competition_2026_d.auto_start import (
        AutoStartGate,
        TASK1_MODE,
        TASK2_MODE,
        classify_t265_usb,
    )
except ModuleNotFoundError:
    from competition_2026_d.auto_start import (
        AutoStartGate,
        TASK1_MODE,
        TASK2_MODE,
        classify_t265_usb,
    )


class FakeLink:
    def __init__(self):
        self.callbacks = []
        self.published = []
        self.acks = []
        self.stats = SimpleNamespace(
            rx_bytes=0,
            rx_frames=0,
            rx_rejected=0,
            rx_wrong_dest=0,
        )

    def add_callback(self, callback):
        self.callbacks.append(callback)

    def publish(self, message_type, payload, **kwargs):
        self.published.append((message_type, payload, kwargs))
        return len(self.published)

    def acknowledge(self, frame, result=0):
        self.acks.append((frame, result))
        return len(self.acks)

    def feed(self, frame):
        for callback in tuple(self.callbacks):
            callback(frame)


def car_start(mode=TASK1_MODE, session=123, seq=7, flags=None):
    return Frame(
        message_type=MessageType.CAR_START,
        flags=(
            int(Flag.ACK_REQUIRED | Flag.EVENT)
            if flags is None
            else int(flags)
        ),
        source=Device.CAR,
        dest=Device.UAV,
        session_id=session,
        seq=seq,
        sender_ms=100,
        payload=pack_payload(MessageType.CAR_START, (mode, 0x2026D001)),
    )


def make_gate(link=None, ready=True):
    link = link or FakeLink()
    gate = AutoStartGate(
        link,
        config_hash=0x12345678,
        readiness_provider=lambda: ready,
    )
    return link, gate


def test_t265_usb_classification_matches_oled_ids():
    assert classify_t265_usb("Bus 1 ID 8087:0B37 Intel") == "ready"
    assert classify_t265_usb("Bus 1 ID 03e7:2150 Movidius") == "need_replug"
    assert classify_t265_usb("") == "not_found"


def test_ready_advertises_both_tasks_only_when_shared_preflight_passes():
    ready_link, ready_gate = make_gate(ready=True)
    assert ready_gate.wait_selection(timeout_s=0.01) is None
    assert len(ready_link.published) == 1
    message_type, payload, kwargs = ready_link.published[0]
    task_mask, ready_bits, _ = unpack_payload(MessageType.UAV_READY, payload)
    assert message_type == MessageType.UAV_READY
    assert task_mask == 0x03
    assert ready_bits == 0x0F
    assert kwargs["session_id"] == 0

    blocked_link, blocked_gate = make_gate(ready=False)
    assert blocked_gate.wait_selection(timeout_s=0.01) is None
    assert blocked_link.published == []


def test_task1_selection_is_not_acked_until_red_gate_confirms():
    link, gate = make_gate()
    frame = car_start(TASK1_MODE)

    link.feed(frame)
    selection = gate.wait_selection(timeout_s=0.1)

    assert selection.task_mode == TASK1_MODE
    assert link.acks == []
    assert gate.confirm_selection(selection) == 1
    assert link.acks == [(frame, 0)]
    assert gate.session_id == frame.session_id
    assert gate.selected_task_mode == TASK1_MODE


def test_task2_selection_and_duplicate_are_idempotent():
    link, gate = make_gate()
    first = car_start(TASK2_MODE, session=456, seq=10)
    duplicate = car_start(TASK2_MODE, session=456, seq=11)

    link.feed(first)
    selection = gate.wait_selection(timeout_s=0.1)
    gate.confirm_selection(selection)
    link.feed(duplicate)

    assert [result for _, result in link.acks] == [0, 0]
    assert gate.selected_task_mode == TASK2_MODE


def test_unknown_task_and_zero_session_are_rejected():
    link, gate = make_gate()
    unknown = car_start(mode=3)
    zero_session = car_start(mode=TASK1_MODE, session=0, seq=8)

    link.feed(unknown)
    link.feed(zero_session)

    assert [result for _, result in link.acks] == [1, 1]
    assert gate.rejected_start_frames == 2


def test_missing_event_flag_is_rejected():
    link, gate = make_gate()
    frame = car_start(flags=Flag.ACK_REQUIRED)

    link.feed(frame)

    assert link.acks == [(frame, 1)]
    assert gate.rejected_start_frames == 1


def test_selection_can_be_rejected_and_retried():
    link, gate = make_gate()
    first = car_start(TASK1_MODE, session=1, seq=1)
    second = car_start(TASK2_MODE, session=2, seq=2)

    link.feed(first)
    pending = gate.wait_selection(timeout_s=0.1)
    gate.reject_selection(pending)
    link.feed(second)
    retried = gate.wait_selection(timeout_s=0.1)

    assert retried.task_mode == TASK2_MODE
    assert link.acks == [(first, 1)]
