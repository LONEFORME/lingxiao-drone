import math

from Lcode.inventory_planner import InventoryPlanner, WaypointKind
from Lcode.warehouse_model import FaceId, FlightPoint, WarehouseConfig, WarehouseModel


def test_coordinate_axes_and_confirmed_lower_bypass():
    config = WarehouseConfig()
    assert config.flight_y(config.start_u_m + 1.0) == 1.0
    assert config.flight_x(config.start_v_m + 1.0) == -1.0
    assert config.lower_bypass_x_m == 0.30


def test_builds_24_unique_slots_and_expected_face_directions():
    model = WarehouseModel()
    assert len(model.slots) == 24
    assert set(model.slots) == {
        f"{face}{number}" for face in "ABCD" for number in range(1, 7)
    }
    points = {(slot.point.x, slot.point.y, slot.point.z) for slot in model.slots.values()}
    assert len(points) == 24
    assert model.faces[FaceId.A].look_y_sign == +1
    assert model.faces[FaceId.B].look_y_sign == -1
    assert model.faces[FaceId.C].look_y_sign == +1
    assert model.faces[FaceId.D].look_y_sign == -1


def test_d_face_scan_line_is_shifted_negative_y_by_twenty_centimeters_only():
    model = WarehouseModel()
    assert math.isclose(model.faces[FaceId.B].scan_y_m, 1.65)
    assert math.isclose(model.faces[FaceId.D].scan_y_m, 3.45)
    assert all(
        math.isclose(model.slots[f"D{number}"].point.y, 3.45)
        for number in range(1, 7)
    )


def test_opposite_faces_reverse_visible_column_labels():
    model = WarehouseModel()
    assert model.slots["A1"].point.x == -1.75
    assert model.slots["A3"].point.x == -0.70
    assert model.slots["B1"].point.x == -0.70
    assert model.slots["B3"].point.x == -1.75


def test_full_plan_has_all_slots_once_and_fixed_gimbal_per_face():
    planner = InventoryPlanner()
    route = planner.plan_full_inventory()
    inspections = [p for p in route if p.kind == WaypointKind.INSPECT]
    assert len(inspections) == 24
    assert len({p.slot_label for p in inspections}) == 24
    assert route[0].kind == WaypointKind.TAKEOFF
    assert route[-1].kind == WaypointKind.LAND

    set_gimbal = [p for p in route if p.kind == WaypointKind.SET_GIMBAL]
    assert [p.face for p in set_gimbal] == list(InventoryPlanner.FULL_FACE_ORDER)
    assert [p.gimbal_angle_deg for p in set_gimbal] == [0.0, 180.0, 0.0, 180.0]
    assert 0 < planner.route_length_m(route) < 28

    assert [p.slot_label for p in inspections] == [
        "A1", "A2", "A3", "A6", "A5", "A4",
        "B6", "B5", "B4", "B1", "B2", "B3",
        "C1", "C2", "C3", "C6", "C5", "C4",
        "D6", "D5", "D4", "D1", "D2", "D3",
    ]


def test_face_scan_starts_at_previous_face_height_and_bypass_keeps_it():
    planner = InventoryPlanner()
    route = planner.plan_full_inventory()
    first_inspect_z = [
        next(p.point.z for p in route if p.kind == WaypointKind.INSPECT and p.face == face)
        for face in InventoryPlanner.FULL_FACE_ORDER
    ]
    assert first_inspect_z == [1.25, 0.85, 1.25, 0.85]

    upper_bypass_z = [
        p.point.z
        for p in route
        if p.kind == WaypointKind.TRANSIT and math.isclose(p.point.x, -2.80)
    ]
    assert upper_bypass_z == [0.85, 0.85, 0.85, 0.85]


def test_full_plan_forces_a_to_b_through_upper_bypass():
    planner = InventoryPlanner()
    route = planner.plan_full_inventory()
    a4_index = next(i for i, p in enumerate(route) if p.slot_label == "A4")
    b6_index = next(i for i, p in enumerate(route) if p.slot_label == "B6")
    between = route[a4_index + 1 : b6_index]
    transit = [p.point for p in between if p.kind == WaypointKind.TRANSIT]
    expected = [
        (-2.80, 0.05, 0.85),
        (-2.80, 1.65, 0.85),
        (-1.75, 1.65, 0.85),
    ]
    assert len(transit) == len(expected)
    assert all(
        math.isclose(point.x, x)
        and math.isclose(point.y, y)
        and math.isclose(point.z, z)
        for point, (x, y, z) in zip(transit, expected)
    )


def test_full_plan_forces_c_to_d_through_upper_bypass():
    planner = InventoryPlanner()
    route = planner.plan_full_inventory()
    c4_index = next(i for i, p in enumerate(route) if p.slot_label == "C4")
    d6_index = next(i for i, p in enumerate(route) if p.slot_label == "D6")
    between = route[c4_index + 1 : d6_index]
    transit = [p.point for p in between if p.kind == WaypointKind.TRANSIT]
    expected = [
        (-2.80, 2.05, 0.85),
        (-2.80, 3.45, 0.85),
        (-1.75, 3.45, 0.85),
    ]
    assert len(transit) == len(expected)
    assert all(
        math.isclose(point.x, x)
        and math.isclose(point.y, y)
        and math.isclose(point.z, z)
        for point, (x, y, z) in zip(transit, expected)
    )


def test_target_on_far_side_uses_confirmed_lower_bypass_when_shorter():
    planner = InventoryPlanner()
    route = planner.plan_target("D3")
    transit_x = [p.point.x for p in route if p.kind == WaypointKind.TRANSIT]
    assert any(math.isclose(x, 0.30) for x in transit_x)
    assert [p.slot_label for p in route if p.kind == WaypointKind.INSPECT] == ["D3"]


def test_same_x_crossing_uses_upper_bypass_when_it_is_shorter():
    planner = InventoryPlanner()
    start_slot = planner.model.slots["A1"]
    end_slot = planner.model.slots["B3"]
    start = FlightPoint(start_slot.point.x, start_slot.point.y, planner.model.config.cruise_z_m)
    end_point = FlightPoint(end_slot.point.x, end_slot.point.y, planner.model.config.cruise_z_m)

    assert math.isclose(start.x, end_point.x)
    assert math.isclose(planner._choose_bypass_x(start, end_point), -2.80)



def test_safe_return_from_each_face_uses_required_bypass():
    planner = InventoryPlanner()
    for label in ("A1", "B1", "C1", "D1"):
        slot = planner.model.slots[label]
        route = planner.plan_safe_return(slot.point)
        assert route[-1].kind == WaypointKind.LAND_APPROACH
        crossed = planner._crossed_shelf_planes(
            slot.point.y, planner.model.landing_approach.y
        )
        if crossed:
            bypass_xs = {
                planner.model.config.lower_bypass_x_m,
                planner.model.config.upper_bypass_x_m,
            }
            assert any(w.point.x in bypass_xs for w in route[:-1])


def test_safe_return_ends_at_land_approach_without_land_final():
    planner = InventoryPlanner()
    route = planner.plan_safe_return(FlightPoint(-1.75, 0.05, 1.25))
    assert route[-1].kind == WaypointKind.LAND_APPROACH
    assert route[-1].point == planner.model.landing_approach
    assert all(w.kind != WaypointKind.LAND for w in route)


def test_safe_return_without_shelf_crossing_is_direct():
    planner = InventoryPlanner()
    approach = planner.model.landing_approach
    current = FlightPoint(approach.x + 0.2, approach.y, approach.z)
    route = planner.plan_safe_return(current)
    assert len(route) == 1
    assert route[0].point == approach
    assert route[0].kind == WaypointKind.LAND_APPROACH


def test_safe_return_ignores_small_cruise_height_measurement_error():
    planner = InventoryPlanner()
    slot = planner.model.slots["A1"]
    current = FlightPoint(slot.point.x, slot.point.y, planner.model.config.cruise_z_m - 0.01)
    route = planner.plan_safe_return(current)

    assert route[0].point.x != current.x
    assert route[0].point.y == current.y
    assert route[0].kind == WaypointKind.TRANSIT


def test_safe_return_from_lower_slot_climbs_before_horizontal_transit():
    planner = InventoryPlanner()
    slot = planner.model.slots["A4"]
    route = planner.plan_safe_return(slot.point)

    assert route[0].point == FlightPoint(
        slot.point.x,
        slot.point.y,
        planner.model.config.cruise_z_m,
    )
    assert route[0].kind == WaypointKind.TRANSIT


def test_invalid_bypass_inside_shelf_is_rejected():
    try:
        WarehouseConfig(lower_bypass_x_m=-0.50)
    except ValueError as exc:
        assert "下端通道" in str(exc)
    else:
        raise AssertionError("货架范围内的下端通道应被拒绝")
