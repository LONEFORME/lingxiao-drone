from pathlib import Path

import pytest

from route_only_main import configure_route_only_environment, validate_route


def test_validate_full_route():
    route = Path(__file__).with_name("router_full_inventory_test.txt")
    points = validate_route(route)
    assert len(points) == 40
    assert max(point[2] for point in points) == 1.4
    assert sum(abs(point[0] - 0.30) < 1e-9 for point in points) == 4
    assert points[-1] == (-2.5, 3.5, 0.2)


def test_validate_route_rejects_wrong_count(tmp_path):
    route = tmp_path / "route.txt"
    route.write_text("0,0,1.4\n", encoding="utf-8")
    with pytest.raises(ValueError, match="40"):
        validate_route(route)


def test_route_only_configures_cruise_profile(monkeypatch):
    monkeypatch.delenv("DRONE_NAV_PROFILE", raising=False)
    monkeypatch.delenv("DRONE_CRUISE_CONFIRM_CYCLES", raising=False)
    monkeypatch.delenv("DRONE_CRUISE_REQUIRE_Z", raising=False)

    configure_route_only_environment()

    assert __import__("os").environ["DRONE_NAV_PROFILE"] == "cruise"
    assert __import__("os").environ["DRONE_CRUISE_CONFIRM_CYCLES"] == "2"
    assert __import__("os").environ["DRONE_CRUISE_REQUIRE_Z"] == "1"
