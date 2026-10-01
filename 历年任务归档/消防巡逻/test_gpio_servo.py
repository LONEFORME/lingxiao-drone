import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from Lcode.gpio_servo import (
    set_servo_angle,
    _angle_to_duty_ns,
    _PWM_BASE,
    SERVO_ANGLE_OPEN,
    SERVO_ANGLE_CLOSED,
)


def _pwm_available():
    return os.path.isdir(_PWM_BASE)


class TestAngleToDutyNs:
    def test_maps_range_correctly(self):
        assert _angle_to_duty_ns(0) == 500_000
        assert _angle_to_duty_ns(180) == 2_500_000
        assert _angle_to_duty_ns(90) == 1_500_000

    def test_clamps_out_of_range(self):
        assert _angle_to_duty_ns(-10) == 500_000
        assert _angle_to_duty_ns(200) == 2_500_000


class TestSetServoAngle:
    def test_does_not_raise_and_returns_bool(self):
        """两边环境都要能跑：本机(Windows)没有/sys/class/pwm，降级返回False；
        板载环境有sysfs PWM时走真实分支返回True(2026-07-17台架实测确认真的
        转动了舵机，且验证了先period后enable的写入顺序不会触发EINVAL)。
        不管哪边，都不应该抛异常。"""
        result = set_servo_angle(SERVO_ANGLE_OPEN)
        assert isinstance(result, bool)
        set_servo_angle(SERVO_ANGLE_CLOSED)  # 不留舵机停在打开位置

    def test_matches_pwm_availability_on_this_machine(self):
        expected = _pwm_available()
        result = set_servo_angle(SERVO_ANGLE_OPEN)
        set_servo_angle(SERVO_ANGLE_CLOSED)
        assert result is expected
