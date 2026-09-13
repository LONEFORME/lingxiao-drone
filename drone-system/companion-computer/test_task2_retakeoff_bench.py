import math

from .task2_retakeoff_bench import (
    FcSnapshot,
    _build_parser,
    _set_fc_command,
    _state_matches,
)


def _snapshot(*, unlock, pwm, fc_age=0.01, pwm_age=0.1):
    return FcSnapshot(
        unlock_sta=unlock,
        mission_stage=5,
        motor_pwm_mask=pwm,
        fc_age_s=fc_age,
        pwm_age_s=pwm_age,
    )


def test_locked_requires_unlock_zero_and_fresh_zero_motor_bits():
    assert _state_matches(
        _snapshot(unlock=0, pwm=0x00),
        unlocked=False,
        max_fc_age_s=0.25,
        max_pwm_age_s=2.0,
    )
    assert not _state_matches(
        _snapshot(unlock=0, pwm=0x0F),
        unlocked=False,
        max_fc_age_s=0.25,
        max_pwm_age_s=2.0,
    )
    assert not _state_matches(
        _snapshot(unlock=1, pwm=0x00),
        unlocked=False,
        max_fc_age_s=0.25,
        max_pwm_age_s=2.0,
    )


def test_land_timeout_flag_does_not_count_as_motor_pwm():
    assert _snapshot(unlock=0, pwm=0x10).motor_bits == 0
    assert _state_matches(
        _snapshot(unlock=0, pwm=0x10),
        unlocked=False,
        max_fc_age_s=0.25,
        max_pwm_age_s=2.0,
    )


def test_unlocked_requires_nonzero_fresh_motor_bits():
    assert _state_matches(
        _snapshot(unlock=1, pwm=0x0F),
        unlocked=True,
        max_fc_age_s=0.25,
        max_pwm_age_s=2.0,
    )
    assert not _state_matches(
        _snapshot(unlock=1, pwm=0x00),
        unlocked=True,
        max_fc_age_s=0.25,
        max_pwm_age_s=2.0,
    )


def test_stale_or_missing_telemetry_never_confirms_state():
    assert not _state_matches(
        _snapshot(unlock=0, pwm=0, fc_age=math.inf),
        unlocked=False,
        max_fc_age_s=0.25,
        max_pwm_age_s=2.0,
    )
    assert not _state_matches(
        _snapshot(unlock=0, pwm=None, pwm_age=math.inf),
        unlocked=False,
        max_fc_age_s=0.25,
        max_pwm_age_s=2.0,
    )


def test_fc_command_reset_preserves_frame_and_clears_only_task_fields():
    se_fc = [170, 2, 1, 80, 20, 120, 60, 101, 51, 99, 255]
    _set_fc_command(
        se_fc,
        task_sta=0,
        next_task_sign=0,
        height_cm=0,
    )
    assert se_fc == [170, 2, 0, 51, 51, 0, 51, 0, 51, 99, 255]


def test_default_pwm_freshness_allows_measured_2p5_second_frame_period():
    args = _build_parser().parse_args(["--propellers-removed"])
    assert args.max_pwm_age_s == 4.0
    assert _state_matches(
        _snapshot(unlock=0, pwm=0, pwm_age=2.6),
        unlocked=False,
        max_fc_age_s=args.max_fc_age_s,
        max_pwm_age_s=args.max_pwm_age_s,
    )
