import Mission_GPT as mg
from Mission_GPT import mission


def _make_mission():
    item = mission([0] * 14, [0] * 11, realsense_obj=None, serial_fc_ref=None)
    item.targets = [[0.0, 0.0, 1.5], [0.0, 0.0, 0.15]]
    return item


class _StableRealsense:
    def get_tracking_confidence(self):
        return 3

    def get_velocity(self):
        return [0.0, 0.0, 0.0]

    def get_orientation(self):
        return [0.0, 0.0, 0.0]


def test_observation_waypoint_uses_global_hold(monkeypatch):
    monkeypatch.setattr(mg, "arrival_hold_s", 15.0)
    item = _make_mission()
    item.target_index = 0
    assert item._waypoint_hold_s() == 15.0
    assert item._waypoint_timeout_s("precision") == mg.arrival_timeout_max


def test_final_waypoint_has_no_hold_but_keeps_descent_timeout(monkeypatch):
    monkeypatch.setattr(mg, "final_waypoint_hold_s", 0.0)
    monkeypatch.setattr(mg, "final_waypoint_timeout_s", 20.0)
    item = _make_mission()
    item.target_index = 1
    assert item._waypoint_hold_s() == 0.0
    assert item._waypoint_timeout_s("precision") == 20.0


def test_empty_route_is_not_final_waypoint():
    item = _make_mission()
    item.targets = []
    item.target_index = 0
    assert item._is_final_waypoint() is False


def test_final_waypoint_advances_on_confirmation_without_extra_hold(monkeypatch):
    monkeypatch.setattr(mg, "final_waypoint_hold_s", 0.0)
    item = _make_mission()
    item.target_index = 1
    item.t265_ok = True
    item.realsense = _StableRealsense()
    for _ in range(mg.arrival_confirm_need):
        item.navigate([0.0, 0.0, 0.15], 0.0)
    assert item.target_index == 2
