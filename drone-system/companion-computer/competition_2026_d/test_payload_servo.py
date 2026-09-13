from drone_control.competition_2026_d.Lcode.gpio_servo import (
    SERVO_ANGLE_CLOSED,
    SERVO_ANGLE_OPEN,
    _angle_to_duty_ns,
)
from drone_control.competition_2026_d.payload_actuator import ActuatorState
from drone_control.competition_2026_d.payload_servo import build_payload_actuator


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def test_servo_angles_match_real_mechanism():
    assert SERVO_ANGLE_CLOSED == 180
    assert SERVO_ANGLE_OPEN == 0
    assert _angle_to_duty_ns(0) == 500_000
    assert _angle_to_duty_ns(180) == 2_500_000


def test_payload_locks_releases_waits_and_relocks():
    clock = Clock()
    angles = []

    def writer(angle):
        angles.append(angle)
        return True

    actuator, hardware = build_payload_actuator(
        release_hold_s=1.0, clock=clock, angle_writer=writer
    )
    assert hardware.ready
    assert angles == [SERVO_ANGLE_CLOSED]
    assert actuator.release_once()
    assert angles[-1] == SERVO_ANGLE_OPEN
    clock.now = 0.9
    assert actuator.poll() == ActuatorState.RELEASING
    clock.now = 1.01
    assert actuator.poll() == ActuatorState.RELEASED
    assert angles[-1] == SERVO_ANGLE_CLOSED


def test_failed_release_write_becomes_uncertain():
    clock = Clock()

    def writer(angle):
        return angle == SERVO_ANGLE_CLOSED

    actuator, _hardware = build_payload_actuator(
        release_hold_s=0.2, clock=clock, angle_writer=writer
    )
    assert actuator.release_once()
    clock.now = 1.1
    assert actuator.poll() == ActuatorState.UNCERTAIN
