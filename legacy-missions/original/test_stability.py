"""Unit tests for the stability refactor (Lpid D term, Mission height ramp,
closed-loop takeoff).

Run on the Raspberry Pi (where simple_pid, colorlog, pyserial, numpy, wiringpi
are installed):

    cd drone_control && python -m pytest test_stability.py -v

On a dev machine lacking the Raspberry-Pi-only `wiringpi` module, stub it before
import, e.g. at the top of a conftest.py:

    import sys, types; sys.modules.setdefault('wiringpi', types.ModuleType('wiringpi'))
"""
import pytest
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))


# ════════════════════ Lpid tests ════════════════════

class TestLpid:
    def test_xy_pid_default_d_is_0_05(self):
        from Lcode.Lpid import PID
        pid = PID(type=0)
        assert pid.xyd == 0.05

    def test_xy_pid_simple_pid_receives_d_term(self):
        from Lcode.Lpid import PID
        pid = PID(type=0)
        assert pid.pid.Kd == pytest.approx(0.05)

    def test_custom_xy_params_override_defaults(self):
        from Lcode.Lpid import PID
        pid = PID(type=0, target=0.5, p=1.0, i=0.01, d=0.1)
        assert pid.pid.Kp == pytest.approx(1.0)
        assert pid.pid.Ki == pytest.approx(0.01)
        assert pid.pid.Kd == pytest.approx(0.1)

    def test_yaw_pid_params_unchanged(self):
        from Lcode.Lpid import PID
        pid = PID(type=1)
        assert pid.yawp == 1.5
        assert pid.yawi == 0.0
        assert pid.yawd == 0.3

    def test_custom_yaw_params(self):
        from Lcode.Lpid import PID
        pid = PID(type=1, p=2.0, d=0.5)
        assert pid.pid.Kp == pytest.approx(2.0)
        assert pid.pid.Kd == pytest.approx(0.5)


# ════════════════════ Mission ramp tests ════════════════════

from unittest.mock import MagicMock, patch


def make_mission():
    """Minimal mission instance: no real serial/T265/K230."""
    from Mission_GPT import mission as MissionClass
    re_fc = [0, 0, 0, 0, 0]
    se_fc = [170, 2, 0, 128, 128, 120, 128, 0, 128, 0, 255]
    re_dmz = [('A9', 'B1'), ('A10', 'B2'), ('A11', 'B3')]
    se_dmz = [0xAA, 0, 0xFF, 0, 0xFF]
    realsense = MagicMock()
    k230 = MagicMock()
    serial_fc = MagicMock()
    serial_fc._last_laser_height_cm = 0.0
    waypoints = [[0.0, 0.0, 1.0], [0.5, 0.0, 1.2]]
    with patch.object(MissionClass, 'load_waypoints', return_value=waypoints):
        m = MissionClass(re_fc, se_fc, re_dmz, se_dmz, realsense, k230, serial_fc)
    return m


class TestMissionRamp:
    def test_ramp_z_initialized_to_zero(self):
        m = make_mission()
        assert m._ramp_z_cm == 0.0

    def test_step_ramp_increases_toward_target(self):
        m = make_mission()
        m._ramp_z_cm = 90.0
        m._step_ramp_z(100)
        assert m._ramp_z_cm == pytest.approx(91.5)

    def test_step_ramp_decreases_toward_target(self):
        m = make_mission()
        m._ramp_z_cm = 110.0
        m._step_ramp_z(100)
        assert m._ramp_z_cm == pytest.approx(108.5)

    def test_step_ramp_clamps_when_within_step(self):
        m = make_mission()
        m._ramp_z_cm = 99.2
        m._step_ramp_z(100)
        assert m._ramp_z_cm == pytest.approx(100.0)

    def test_navigate_sends_ramp_z_not_direct_target(self):
        import time
        m = make_mission()
        m._ramp_z_cm = 90.0
        m.target_index = 0
        m.t265_ok = True
        m.arrival_start_time = time.time()
        m.realsense.get_tracking_confidence.return_value = 3
        m.realsense.get_velocity.return_value = [0.0, 0.0, 0.0]

        m.navigate([0.0, 0.0, 0.9], 0.0)

        # se_fc[5] must reflect ramp value (91), not direct target_z (100)
        assert m.se_fc[5] == 91


# ════════════════════ Takeoff tests ════════════════════

class TestTakeoff:
    def test_takeoff_sets_task_sta_to_1(self):
        """se_fc[2] must be set to 1 immediately."""
        m = make_mission()
        m.t265_ok = False

        tick = [0.0]
        def fake_time():
            tick[0] += 2.0          # advance 2 s per call → timeout in ~8 calls
            return tick[0]

        with patch('Mission_GPT.time.sleep', lambda _: None), \
             patch('Mission_GPT.time.time', fake_time):
            m.takeoff()

        assert m.se_fc[2] == 1

    def test_takeoff_transitions_to_navigate(self):
        """State must be NAVIGATE after takeoff() returns."""
        m = make_mission()
        m.t265_ok = False

        tick = [0.0]
        def fake_time():
            tick[0] += 2.0
            return tick[0]

        with patch('Mission_GPT.time.sleep', lambda _: None), \
             patch('Mission_GPT.time.time', fake_time):
            m.takeoff()

        assert m.state == "NAVIGATE"

    def test_takeoff_initializes_ramp_z_to_first_waypoint(self):
        """_ramp_z_cm must equal targets[0][2]*100 when takeoff exits."""
        m = make_mission()
        m.t265_ok = False

        tick = [0.0]
        def fake_time():
            tick[0] += 2.0
            return tick[0]

        with patch('Mission_GPT.time.sleep', lambda _: None), \
             patch('Mission_GPT.time.time', fake_time):
            m.takeoff()

        assert m._ramp_z_cm == pytest.approx(100.0)   # targets[0][2]=1.0 m → 100 cm

    def test_takeoff_exits_early_on_height_confirmed(self):
        """If laser height matches target for 10 frames, exit before timeout."""
        m = make_mission()
        m.t265_ok = False
        m.serial_fc_ref._last_laser_height_cm = 1.0   # 1.0 m = 100 cm (target)

        call_count = [0]
        tick = [0.0]
        def fake_time():
            tick[0] += 0.03        # realistic 30 ms per frame — stays well within 15 s
            return tick[0]

        def fake_sleep(t):
            call_count[0] += 1

        with patch('Mission_GPT.time.sleep', fake_sleep), \
             patch('Mission_GPT.time.time', fake_time):
            m.takeoff()

        assert m.state == "NAVIGATE"
        assert call_count[0] <= 20   # must exit in ≤ 20 frames (10 confirm + margin)
