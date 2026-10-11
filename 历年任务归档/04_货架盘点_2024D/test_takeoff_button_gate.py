"""一键起飞按钮门禁测试：验证硬件对象只在按键后创建。"""
import inspect
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import main as main_module
from Mission_GPT import TAKEOFF_WARN_LED_DURATION_S, mission


class FakeButton:
    available = True
    pressed_values = [False, True]
    last_instance = None

    def __init__(self):
        self.stopped = False
        self._pressed_values = iter(self.pressed_values)
        type(self).last_instance = self

    def start(self):
        return self.available

    def was_pressed(self):
        return next(self._pressed_values)

    def stop(self):
        self.stopped = True


def test_green_waits_then_turns_off_before_hardware_init(monkeypatch):
    colors = []
    sleeps = []
    FakeButton.available = True
    FakeButton.pressed_values = [False, True]
    monkeypatch.setattr("Lcode.gpio_button.GpioButton", FakeButton)
    monkeypatch.setattr(
        "Lcode.gpio_led.set_rgb_led",
        lambda color: colors.append(color) or True,
    )
    monkeypatch.setattr(main_module.time, "sleep", sleeps.append)

    assert main_module.wait_for_start_button() is True
    assert colors == ["G", "OFF"]
    assert sleeps == [main_module.START_BUTTON_POLL_S]
    assert TAKEOFF_WARN_LED_DURATION_S == 5.0
    assert FakeButton.last_instance.stopped is True


def test_button_unavailable_fails_closed_and_stops(monkeypatch):
    colors = []
    FakeButton.available = False
    FakeButton.pressed_values = []
    monkeypatch.setattr("Lcode.gpio_button.GpioButton", FakeButton)
    monkeypatch.setattr(
        "Lcode.gpio_led.set_rgb_led",
        lambda color: colors.append(color) or True,
    )

    assert main_module.wait_for_start_button() is False
    assert colors == []
    assert FakeButton.last_instance.stopped is True


def test_green_led_failure_fails_closed_and_cleans_up(monkeypatch):
    colors = []
    FakeButton.available = True
    FakeButton.pressed_values = [True]
    monkeypatch.setattr("Lcode.gpio_button.GpioButton", FakeButton)

    def set_led(color):
        colors.append(color)
        return color == "OFF"

    monkeypatch.setattr("Lcode.gpio_led.set_rgb_led", set_led)

    assert main_module.wait_for_start_button() is False
    assert colors == ["G", "OFF"]
    assert FakeButton.last_instance.stopped is True


def test_mission_warns_red_after_t265_check_before_takeoff_state():
    source = inspect.getsource(mission.start)
    assert "_wait_for_takeoff_button" not in source
    assert source.index("_blink_warning_led") < source.index("self.task_running = True")


def test_main_does_not_create_hardware_when_button_gate_fails(monkeypatch):
    events = []
    monkeypatch.setattr(main_module, "WAREHOUSE_MISSION_READY", True)
    monkeypatch.setattr(
        main_module,
        "wait_for_start_button",
        lambda: events.append("button") or False,
    )
    monkeypatch.setattr(
        main_module,
        "t265_class",
        lambda: events.append("t265"),
    )

    main_module.main()
    assert events == ["button"]


def test_unimplemented_warehouse_entry_stays_locked(monkeypatch):
    events = []
    monkeypatch.setattr(main_module, "WAREHOUSE_MISSION_READY", False)
    monkeypatch.setattr(
        main_module,
        "wait_for_start_button",
        lambda: events.append("button") or True,
    )
    monkeypatch.setattr(
        main_module,
        "t265_class",
        lambda: events.append("t265"),
    )

    main_module.main()
    assert events == []


def test_takeoff_does_not_repeat_warning_gate():
    assert "_blink_warning_led" not in inspect.getsource(mission.takeoff)
