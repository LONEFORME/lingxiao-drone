import math
import time

import pytest

import Mission_GPT as mg
from Lcode.Lprotocol import Serial_fc
from Lcode.global_variable import (
    fc_frame_counter,
    fc_last_rx_monotonic,
    fc_last_rx_time,
    lock,
    sp_side,
)
from Mission_GPT import HeadingFeedbackError, mission


class FakeRealsense:
    def __init__(self, confidence=3):
        self.confidence = confidence

    def get_tracking_confidence(self):
        return self.confidence

    def get_velocity(self):
        return [0.0, 0.0, 0.0]


@pytest.fixture(autouse=True)
def restore_fc_globals():
    with lock:
        old = (
            fc_frame_counter.value,
            fc_last_rx_monotonic.value,
            fc_last_rx_time.value,
        )
        fc_frame_counter.value = 0
        fc_last_rx_monotonic.value = 0.0
        fc_last_rx_time.value = 0.0
    yield
    with lock:
        fc_frame_counter.value = old[0]
        fc_last_rx_monotonic.value = old[1]
        fc_last_rx_time.value = old[2]


def _publish_fc_yaw(m, yaw_deg, *, frame_increment=1, received_at=None):
    with lock:
        m.re_fc[3] = int(round(yaw_deg * 100))
        fc_frame_counter.value += frame_increment
        fc_last_rx_monotonic.value = (
            time.monotonic() if received_at is None else float(received_at)
        )


def _fc_mission(fc_yaw_deg=20.0, t265_yaw_deg=10.0):
    m = mission([0] * 14, [0] * 11, realsense_obj=None, serial_fc_ref=None)
    m.heading_source = "fc"
    m.t265_ok = True
    m.realsense = FakeRealsense()
    with lock:
        fc_frame_counter.value = mg.FC_HEADING_MIN_PREUNLOCK_FRAMES
    _publish_fc_yaw(m, fc_yaw_deg, frame_increment=0)
    m._heading_status = m._arm_heading_hold(math.radians(t265_yaw_deg))
    m._ramp_z_cm = 125.0
    return m


def test_default_heading_source_is_t265():
    m = mission([0] * 14, [0] * 11, realsense_obj=None, serial_fc_ref=None)

    assert m.heading_source == "t265"


def test_explicit_fc_heading_source_latches_fc_yaw():
    m = _fc_mission(fc_yaw_deg=24.5, t265_yaw_deg=-13.0)

    assert m.heading_hold.target_deg == pytest.approx(24.5)
    assert m._heading_t265_reference_deg == pytest.approx(-13.0)
    assert m._heading_fc_reference_deg == pytest.approx(24.5)


def test_t265_yaw_drift_does_not_command_or_fault_when_fc_yaw_is_fixed():
    m = _fc_mission(fc_yaw_deg=20.0, t265_yaw_deg=0.0)

    status = m._update_heading_hold(math.radians(30.0), confidence=3)

    assert status.command_dps == 0
    assert status.error_deg == pytest.approx(0.0)
    assert status.fault_reason is None
    assert m._heading_source_disagreement_deg == pytest.approx(30.0)


def test_command_follows_fc_yaw_only_and_uses_fc_command_polarity():
    m = _fc_mission(fc_yaw_deg=20.0, t265_yaw_deg=0.0)
    _publish_fc_yaw(m, 27.5)

    status = m._update_heading_hold(math.radians(-40.0), confidence=3)

    assert status.error_deg == pytest.approx(-7.5)
    assert status.command_dps == 2
    assert status.fault_reason is None


def test_exact_eight_degree_error_triggers_existing_heading_fault():
    m = _fc_mission(fc_yaw_deg=20.0, t265_yaw_deg=0.0)
    _publish_fc_yaw(m, 28.0)

    status = m._update_heading_hold(0.0, confidence=3)

    assert status.error_deg == pytest.approx(-8.0)
    assert status.command_dps == 0
    assert status.fault_reason == "heading_error_-8.00deg_exceeds_limit"


def test_fc_heading_wraps_from_179_to_minus_179_in_short_direction():
    m = _fc_mission(fc_yaw_deg=179.0, t265_yaw_deg=0.0)
    _publish_fc_yaw(m, -179.0)

    status = m._update_heading_hold(0.0, confidence=3)

    assert status.error_deg == pytest.approx(-2.0)
    assert status.command_dps == 1


def test_t265_low_confidence_does_not_disable_valid_fc_heading_feedback():
    m = _fc_mission(fc_yaw_deg=0.0, t265_yaw_deg=0.0)
    _publish_fc_yaw(m, -5.0)

    status = m._update_heading_hold(math.radians(80.0), confidence=0)

    assert status.command_dps == -1
    assert status.degraded_reason is None


def test_preunlock_requires_five_valid_fc_frames():
    m = mission([0] * 14, [0] * 11, realsense_obj=None, serial_fc_ref=None)
    m.heading_source = "fc"
    with lock:
        fc_frame_counter.value = mg.FC_HEADING_MIN_PREUNLOCK_FRAMES - 1
    _publish_fc_yaw(m, 10.0, frame_increment=0)

    with pytest.raises(HeadingFeedbackError, match="not_ready_4_frames"):
        m._arm_heading_hold(0.0)


def test_first_fc_sample_sets_jump_baseline_and_exact_ten_degrees_is_allowed():
    m = _fc_mission(fc_yaw_deg=0.0, t265_yaw_deg=0.0)
    _publish_fc_yaw(m, 10.0)
    status = m._update_heading_hold(0.0, confidence=3)

    assert status.current_deg == pytest.approx(10.0)
    assert status.fault_reason == "heading_error_-10.00deg_exceeds_limit"

    _publish_fc_yaw(m, 20.01)
    with pytest.raises(HeadingFeedbackError, match="fc_heading_jump"):
        m._update_heading_hold(0.0, confidence=3)


def test_fc_heading_minus_180_boundary_is_preserved():
    m = _fc_mission(fc_yaw_deg=-180.0, t265_yaw_deg=0.0)

    assert m.heading_hold.target_deg == pytest.approx(-180.0)
    assert m._fc_heading_last_deg == pytest.approx(-180.0)


def test_non_finite_fc_heading_is_rejected_before_unlock():
    m = mission([0] * 14, [0] * 11, realsense_obj=None, serial_fc_ref=None)
    m.heading_source = "fc"
    with lock:
        fc_frame_counter.value = mg.FC_HEADING_MIN_PREUNLOCK_FRAMES
        m.re_fc[3] = float("nan")
        fc_last_rx_monotonic.value = time.monotonic()

    with pytest.raises(HeadingFeedbackError, match="fc_heading_non_finite"):
        m._arm_heading_hold(0.0)


def test_stale_fc_yaw_pauses_twice_then_lands_with_zero_commands(monkeypatch):
    clock = {"now": 100.0}
    monkeypatch.setattr(mg.time, "monotonic", lambda: clock["now"])
    m = mission([0] * 14, [0] * 11, realsense_obj=None, serial_fc_ref=None)
    m.heading_source = "fc"
    m.t265_ok = True
    m.realsense = FakeRealsense()
    with lock:
        fc_frame_counter.value = mg.FC_HEADING_MIN_PREUNLOCK_FRAMES
    _publish_fc_yaw(m, 0.0, frame_increment=0, received_at=99.5)
    m._heading_status = m._arm_heading_hold(0.0)
    m._ramp_z_cm = 125.0
    m.state = "NAVIGATE"

    # Exactly 0.5 s is still valid. Then three stale control ticks are required.
    m.position_control_tick(m.targets[0], [0.0, 0.0, 1.25], 0.0)
    clock["now"] = 100.001
    first = m.position_control_tick(m.targets[0], [0.0, 0.0, 1.25], 0.0)
    second = m.position_control_tick(m.targets[0], [0.0, 0.0, 1.25], 0.0)
    third = m.position_control_tick(m.targets[0], [0.0, 0.0, 1.25], 0.0)

    assert first["heading_recovery_active"] is True
    assert second["heading_recovery_active"] is True
    assert third["heading_recovery_failed"] is True
    assert m.state == "LAND"
    assert m.se_fc[3] == sp_side
    assert m.se_fc[4] == sp_side
    assert m.se_fc[6] == sp_side


def test_heading_recovery_uses_fc_yaw_not_t265_yaw():
    m = _fc_mission(fc_yaw_deg=0.0, t265_yaw_deg=0.0)
    m.state = "NAVIGATE"
    _publish_fc_yaw(m, -9.0)

    control = m.position_control_tick(
        m.targets[0], [0.0, 0.0, 1.25], math.radians(45.0)
    )

    assert control["heading_recovery_active"] is True
    assert control["yaw_cmd"] == -5
    assert m._heading_status.current_deg == pytest.approx(-9.0)


def test_t265_heading_source_preserves_controller_command_polarity():
    m = _fc_mission(fc_yaw_deg=0.0, t265_yaw_deg=0.0)
    m.heading_source = "t265"

    status = m._update_heading_hold(math.radians(-5.0), confidence=3)

    assert status.error_deg == pytest.approx(5.0)
    assert status.command_dps == 1


class FakeSerial:
    def __init__(self, frame_bytes, owner):
        self._buf = bytearray(frame_bytes)
        self._owner = owner

    def read(self, size=1):
        if not self._buf:
            self._owner.fclisten_running = False
            return b""
        chunk = bytes(self._buf[:size])
        del self._buf[:size]
        return chunk


def _frame1_with_yaw(yaw_x100):
    encoded = int(yaw_x100) & 0xFFFF
    data = bytearray(24)
    data[5] = encoded & 0xFF
    data[6] = (encoded >> 8) & 0xFF
    header = bytes([0x01, 24])
    checksum = (sum(header) + sum(data)) & 0xFF
    return b"\xAA" + header + bytes(data) + bytes([checksum, 0xFF])


def test_frame1_atomically_updates_yaw_monotonic_timestamp_and_counter():
    fc = Serial_fc.__new__(Serial_fc)
    fc.debug_data = {}
    fc.fclisten_running = True
    fc._last_laser_height_cm = 0.0
    fc.ser = FakeSerial(_frame1_with_yaw(-1234), fc)
    rxbuffer = [0] * 14
    before = time.monotonic()

    fc.listen_fc(rxbuffer)

    assert rxbuffer[3] == -1234
    assert fc_frame_counter.value == 1
    assert fc_last_rx_monotonic.value >= before
