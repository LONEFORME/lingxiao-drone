import unittest

from drone_control.competition_2026_d.drop_controller import DropController, DropState
from drone_control.competition_2026_d.dynamic_landing import (
    DynamicLandingController,
    LandingConfig,
    LandingInput,
    LandingState,
)
from drone_control.competition_2026_d.payload_actuator import PayloadActuator
from drone_control.competition_2026_d.t265_hotplug import T265HotplugManager, T265State


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class FakeT265:
    def __init__(self):
        self.last_confidence = 3
        self.started = 0

    def start(self):
        self.started += 1
        return True

    def stop(self):
        pass


class SafetyStateMachinesTest(unittest.TestCase):
    def test_t265_does_not_initialize_before_arm(self):
        clock = Clock()
        created = []
        manager = T265HotplugManager(lambda: True, lambda: created.append(FakeT265()) or created[-1], clock=clock)
        manager.poll()
        self.assertEqual(created, [])
        manager.arm()
        self.assertEqual(manager.poll(), T265State.WAIT_CONFIDENCE)
        clock.advance(1.1)
        self.assertEqual(manager.poll(), T265State.READY)

    def test_drop_is_single_action_and_requires_feedback(self):
        clock = Clock()
        commands = []
        feedback = {"released": False}
        actuator = PayloadActuator(commands.append, lambda: feedback["released"], clock=clock)
        controller = DropController(actuator, clock=clock)
        args = dict(
            vision_fresh=True, vision_quality=90, error_xy_m=(0.01, 0.01),
            relative_velocity_xy_m_s=(0.01, 0.01), altitude_m=1.5,
            inside_drop_region=True,
        )
        self.assertEqual(controller.tick(**args), DropState.STABLE_WINDOW)
        clock.advance(0.31)
        self.assertEqual(controller.tick(**args), DropState.RELEASING)
        feedback["released"] = True
        self.assertEqual(controller.tick(**args), DropState.RELEASED)
        self.assertEqual(len(commands), 2)  # 上电锁定一次、释放一次

    def test_terminal_prediction_has_hard_limit_and_one_reacquire(self):
        clock = Clock()
        controller = DynamicLandingController(clock=clock)
        base = dict(
            relative_height_m=0.15, vertical_speed_m_s=-0.05,
            relative_velocity_xy_m_s=(0.01, 0.01), position_error_xy_m=(0.01, 0.01),
            estimate_uncertainty_m=0.02, visual_usable=True, visual_too_close=False,
            car_motion_fresh=True, roll_deg=1, pitch_deg=1,
            contact_evidence=False, t265_healthy=True,
        )
        controller.tick(LandingInput(**base))  # gate -> terminal
        self.assertEqual(controller.state, LandingState.TERMINAL_PREDICT)
        clock.advance(0.51)
        command = controller.tick(LandingInput(**{**base, "visual_usable": False, "visual_too_close": True}))
        self.assertEqual(command.reason, "terminal_reacquire")
        # 再进入终端段后第二次超时必须中止，不能无限盲降。
        controller.tick(LandingInput(**base))
        self.assertEqual(controller.state, LandingState.TERMINAL_PREDICT)
        clock.advance(0.51)
        command = controller.tick(LandingInput(**{**base, "visual_usable": False, "visual_too_close": True}))
        self.assertEqual(command.state, LandingState.CONTROLLED_ABORT)

    def test_touchdown_requires_independent_contact(self):
        clock = Clock()
        controller = DynamicLandingController(clock=clock)
        controller.state = LandingState.TERMINAL_PREDICT
        controller._state_since = clock()
        controller._terminal_start_height = 0.12
        data = LandingInput(
            relative_height_m=0.11, vertical_speed_m_s=0.0,
            relative_velocity_xy_m_s=(0.0, 0.0), position_error_xy_m=(0.0, 0.0),
            estimate_uncertainty_m=0.01, visual_usable=False, visual_too_close=True,
            car_motion_fresh=True, roll_deg=0, pitch_deg=0,
            contact_evidence=False, t265_healthy=True,
        )
        controller.tick(data)
        self.assertEqual(controller.state, LandingState.TERMINAL_PREDICT)
        with_contact = LandingInput(**{**data.__dict__, "contact_evidence": True})
        controller.tick(with_contact)
        self.assertEqual(controller.state, LandingState.TOUCHDOWN_CANDIDATE)
        clock.advance(0.41)
        command = controller.tick(with_contact)
        self.assertTrue(command.touchdown_confirmed)

    def test_touchdown_holds_five_seconds_before_retakeoff_gate(self):
        clock = Clock()
        controller = DynamicLandingController(clock=clock)
        controller.state = LandingState.TOUCHDOWN_CANDIDATE
        controller._state_since = clock()
        controller._touchdown_since = clock()
        contact = LandingInput(
            relative_height_m=0.10,
            vertical_speed_m_s=0.0,
            relative_velocity_xy_m_s=(0.0, 0.0),
            position_error_xy_m=(0.0, 0.0),
            estimate_uncertainty_m=0.01,
            visual_usable=False,
            visual_too_close=True,
            car_motion_fresh=True,
            roll_deg=0,
            pitch_deg=0,
            contact_evidence=True,
            t265_healthy=True,
        )

        clock.advance(0.41)
        touchdown = controller.tick(contact)
        self.assertEqual(touchdown.state, LandingState.DECK_RIDE)
        self.assertTrue(touchdown.touchdown_confirmed)
        self.assertEqual(touchdown.vertical_speed_m_s, 0.0)

        clock.advance(4.99)
        holding = controller.tick(contact)
        self.assertEqual(holding.state, LandingState.DECK_RIDE)
        self.assertTrue(holding.hold_car_velocity)

        clock.advance(0.02)
        released = controller.tick(contact)
        self.assertEqual(released.state, LandingState.RETAKEOFF_GATE)

    def test_direct_descent_continues_through_vision_loss_until_contact(self):
        clock = Clock()
        controller = DynamicLandingController(
            LandingConfig(direct_descent_after_gate=True),
            clock=clock,
        )
        airborne = LandingInput(
            relative_height_m=1.20,
            vertical_speed_m_s=0.0,
            relative_velocity_xy_m_s=(0.5, 0.5),
            position_error_xy_m=(0.5, 0.5),
            estimate_uncertainty_m=1.0,
            visual_usable=False,
            visual_too_close=False,
            car_motion_fresh=False,
            roll_deg=0.0,
            pitch_deg=0.0,
            contact_evidence=False,
            t265_healthy=True,
        )

        descending = controller.tick(airborne)
        self.assertEqual(descending.state, LandingState.DESCEND_HIGH)
        self.assertEqual(descending.vertical_speed_m_s, -0.30)
        self.assertEqual(descending.reason, "direct_descent_to_contact")

        clock.advance(2.0)
        still_descending = controller.tick(airborne)
        self.assertEqual(still_descending.state, LandingState.DESCEND_HIGH)
        self.assertEqual(still_descending.vertical_speed_m_s, -0.30)

        contact = LandingInput(
            **{
                **airborne.__dict__,
                "relative_height_m": 0.10,
                "vertical_speed_m_s": -0.30,
                "contact_evidence": True,
            }
        )
        touchdown = controller.tick(contact)
        self.assertEqual(touchdown.state, LandingState.TOUCHDOWN_CANDIDATE)
        self.assertEqual(touchdown.vertical_speed_m_s, 0.0)

        clock.advance(0.41)
        confirmed = controller.tick(contact)
        self.assertEqual(confirmed.state, LandingState.DECK_RIDE)
        self.assertTrue(confirmed.touchdown_confirmed)


if __name__ == "__main__":
    unittest.main()
